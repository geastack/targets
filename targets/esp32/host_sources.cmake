# Common ESP32 host facades, compiled into the `main` component of every ESP32
# board target. Single source of truth for the platform-universal host facades:
# a new facade added here — or an app that uses an existing one like `tiles`
# (host/tile_loader.cpp) — lights up on every board without editing each
# target's main/CMakeLists.txt. (Before this, tile_loader.cpp was hand-listed in
# only some targets, so maps failed to link on the others, e.g. amoled-2.06.)
#
# The shared npm resolver defines GEA_HOST as the installed @geastack/host
# package directory before this manifest is included.
#
# Board-HARDWARE-gated facades are intentionally NOT listed here and stay opt-in
# per target (append after the include):
#   - host/camera.cpp: only boards with a camera (the two waveshare-p4 targets).
set(GEA_EMBEDDED_HOST_SOURCES
    "${GEA_HOST}/host/apps.cpp"
    "${GEA_HOST}/host/audio.cpp"
    "${GEA_HOST}/host/ble.cpp"
    "${GEA_HOST}/host/display.cpp"
    "${GEA_HOST}/host/fetch.cpp"
    "${GEA_HOST}/host/geolocation.cpp"
    "${GEA_HOST}/host/gpio.cpp"
    "${GEA_HOST}/host/image.cpp"
    "${GEA_HOST}/host/media.cpp"
    "${GEA_HOST}/host/memory.cpp"
    "${GEA_HOST}/host/input.cpp"
    "${GEA_HOST}/host/imu.cpp"
    "${GEA_HOST}/host/rtc.cpp"
    "${GEA_HOST}/host/rtc_esp.cpp"
    "${GEA_HOST}/host/tile_loader.cpp"
    "${GEA_HOST}/host/timers.cpp"
    "${GEA_HOST}/host/touch.cpp"
    "${GEA_HOST}/host/websocket.cpp"
    "${GEA_HOST}/host/http.cpp"
    "${GEA_HOST}/host/wifi.cpp"
)
