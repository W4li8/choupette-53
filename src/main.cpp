// Selects which app to build via a platformio.ini environment: each
// [env:*] sets exactly one of these defines in build_flags. Pick the app
// at compile/flash time with `pio run -e rgb` / `pio run -e motors`
// (or set default_envs in platformio.ini).
#if defined(APP_RGB)
  #include "apps/rgb_app.inc"
#elif defined(APP_MOTORS)
  #include "apps/motors_app.inc"
#elif defined(APP_LIDAR_POINT)
  #include "apps/lidar_point_app.inc"
#elif defined(APP_LIDAR_MATRIX)
  #include "apps/lidar_matrix_app.inc"
#elif defined(APP_I2C_SCAN)
  #include "apps/i2c_scan_app.inc"
#elif defined(APP_OLED_HELLO)
  #include "apps/oled_hello_app.inc"
#else
  #error "No app selected. Set APP_RGB, APP_MOTORS, APP_LIDAR_POINT, or APP_LIDAR_MATRIX via build_flags in platformio.ini."
#endif
