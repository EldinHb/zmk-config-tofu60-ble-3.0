# Copyright (c) 2025 YANG
# zmk klink indicator
zephyr_library()
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
zephyr_library_sources(src/klink_indicator.c)

# KBD Lock guard, see src/klink_lock_guard.c
zephyr_library_sources(src/klink_lock_guard.c)
zephyr_ld_options(
  -Wl,--wrap=zmk_pm_soft_off
  -Wl,--wrap=raise_zmk_position_state_changed
  -Wl,--wrap=bt_le_adv_start
  -Wl,--wrap=bt_le_adv_stop
)
