// Selects which app to build via a platformio.ini environment: each
// [env:*] sets exactly one of these defines in build_flags. Pick the app
// at compile/flash time with `pio run -e rgb` / `pio run -e motors`
// (or set default_envs in platformio.ini).
#if defined(APP_RGB)
  #include "apps/rgb_app.inc"
#elif defined(APP_MOTORS)
  #include "apps/motors_app.inc"
#else
  #error "No app selected. Set APP_RGB or APP_MOTORS via build_flags in platformio.ini."
#endif
