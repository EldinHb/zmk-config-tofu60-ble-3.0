/*
 * KBD Lock guard: after the keyboard is locked (soft off), only an unlock
 * combination wakes it up for real.
 *
 * Soft off is nRF52 System OFF, and the only way out is a press of the wake
 * key (F, see wakeup_scan in common.overlay), which resets the chip. A key
 * pressed by accident in a bag therefore boots the keyboard. On a boot that
 * follows a lock, every key press is swallowed until UNLOCK_KEYS (and no other
 * key) have been held for UNLOCK_HOLD_MS. If that does not happen within
 * UNLOCK_WINDOW_MS, the keyboard locks itself again.
 *
 * Wake from idle deep sleep (CONFIG_ZMK_SLEEP) is also System OFF, so the two
 * are told apart by a marker in GPREGRET2, which survives System OFF but is
 * cleared by a power-on reset. Switching the battery off and on always boots
 * the keyboard unlocked.
 *
 * Both hooks are linker wraps (see CMakeLists.txt): ZMK raises no event before
 * soft off, and our event listeners run after the keymap's, too late to
 * swallow a key press.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>

#include <zmk/events/position_state_changed.h>
#include <zmk/matrix_transform.h>
#include <zmk/physical_layouts.h>
#include <zmk/pm.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define LOCK_GPREGRET 1 // GPREGRET2; the bootloader uses GPREGRET
#define LOCK_MAGIC 0x4C

#define UNLOCK_HOLD_MS 1000
#define UNLOCK_WINDOW_MS 5000

struct matrix_key {
    uint8_t row;
    uint8_t col;
};

// Matrix row/column, the same on every layout: F and J.
static const struct matrix_key unlock_keys[] = {{2, 4}, {2, 7}};
// Must match wakeup_scan in common.overlay.
static const struct matrix_key wake_key = {2, 4};

static bool guarding;
static bool relock_pending;
static uint64_t held;       // positions currently held
static uint64_t swallowed;  // positions whose press was swallowed, until released

int __real_zmk_pm_soft_off(void);
int __real_raise_zmk_position_state_changed(struct zmk_position_state_changed ev);

int __wrap_zmk_pm_soft_off(void) {
    nrf_power_gpregret_set(NRF_POWER, LOCK_GPREGRET, LOCK_MAGIC);
    int err = __real_zmk_pm_soft_off();
    if (err < 0) {
        nrf_power_gpregret_set(NRF_POWER, LOCK_GPREGRET, 0);
    }
    return err;
}

static uint64_t key_mask(const struct matrix_key *keys, size_t len) {
    struct zmk_physical_layout const *const *layouts;
    size_t count = zmk_physical_layouts_get_list(&layouts);
    int selected = zmk_physical_layouts_get_selected();
    uint64_t mask = 0;

    if (selected < 0 || selected >= count) {
        return 0;
    }
    for (size_t i = 0; i < len; i++) {
        int32_t pos = zmk_matrix_transform_row_column_to_position(
            layouts[selected]->matrix_transform, keys[i].row, keys[i].col);
        if (pos >= 0 && pos < 64) {
            mask |= BIT64(pos);
        }
    }
    return mask;
}

static void relock(void) {
    // The wake key is a level-triggered wake source: going off while it is
    // held would wake straight back up. Wait for its release instead.
    if (held & key_mask(&wake_key, 1)) {
        relock_pending = true;
        return;
    }
    LOG_INF("Lock guard: no unlock combination, locking again");
    int err = zmk_pm_soft_off();
    if (err < 0) {
        // Still on: unlock rather than leave a keyboard that swallows every key.
        LOG_ERR("Lock guard: failed to lock again (%d), unlocking", err);
        guarding = false;
    }
}

static void unlock_work_cb(struct k_work *work) {
    uint64_t combo = key_mask(unlock_keys, ARRAY_SIZE(unlock_keys));

    if (guarding && combo != 0 && held == combo) {
        LOG_INF("Lock guard: unlocked");
        guarding = false;
        nrf_power_gpregret_set(NRF_POWER, LOCK_GPREGRET, 0);
    }
}

static K_WORK_DELAYABLE_DEFINE(unlock_work, unlock_work_cb);

static void window_work_cb(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(window_work, window_work_cb);

static void window_work_cb(struct k_work *work) {
    if (!guarding) {
        return;
    }
    // The unlock combination is being held: let the hold finish.
    if (k_work_delayable_is_pending(&unlock_work)) {
        k_work_schedule(&window_work, K_MSEC(UNLOCK_HOLD_MS));
        return;
    }
    relock();
}

int __wrap_raise_zmk_position_state_changed(struct zmk_position_state_changed ev) {
    if (ev.position >= 64) {
        return guarding ? 0 : __real_raise_zmk_position_state_changed(ev);
    }

    uint64_t bit = BIT64(ev.position);

    if (ev.state) {
        held |= bit;
    } else {
        held &= ~bit;
    }

    if (!guarding) {
        // Releases of keys pressed while guarding were never seen by the keymap.
        if (!ev.state && (swallowed & bit)) {
            swallowed &= ~bit;
            return 0;
        }
        return __real_raise_zmk_position_state_changed(ev);
    }

    if (ev.state) {
        swallowed |= bit;
    } else {
        swallowed &= ~bit;
    }

    if (relock_pending) {
        if (!(held & key_mask(&wake_key, 1))) {
            relock_pending = false;
            relock();
        }
        return 0;
    }

    uint64_t combo = key_mask(unlock_keys, ARRAY_SIZE(unlock_keys));
    if (combo != 0 && held == combo) {
        k_work_reschedule(&unlock_work, K_MSEC(UNLOCK_HOLD_MS));
    } else {
        k_work_cancel_delayable(&unlock_work);
    }
    return 0;
}

// Decide before any driver starts scanning keys.
static int lock_guard_early_init(void) {
    guarding = (nrf_power_gpregret_get(NRF_POWER, LOCK_GPREGRET) == LOCK_MAGIC);
    return 0;
}

SYS_INIT(lock_guard_early_init, PRE_KERNEL_1, 0);

// Timers and work queues are only usable later.
static int lock_guard_init(void) {
    if (guarding) {
        LOG_INF("Lock guard: woke from KBD Lock, waiting for unlock combination");
        k_work_schedule(&window_work, K_MSEC(UNLOCK_WINDOW_MS));
    }
    return 0;
}

SYS_INIT(lock_guard_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
