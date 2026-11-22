# Patches for managed components (superseded)

The patch in this directory is the same esp_video fix as
`../component-patches/0001-esp_video-csi-raw-passthrough-p4-rev-lt-v3.patch`,
which the target's `CMakeLists.txt` applies automatically on every configure
(see [component-patches/README.md](../component-patches/README.md)). Nothing here
needs to be applied by hand; a manual `patch -p1` after a configure finds the
patch as already applied.

## espressif__esp_video-csi-bypass.patch

ESP32-P4 rev < v3.0 (this target is configured for rev v1.3 silicon) cannot do
RAW→RGB565 color conversion in the MIPI-CSI bridge. The patch forces the CSI
controller into 16-bpp pass-through (the inline ISP demosaics instead), sizes
the frame at the unpacked RAW width so the full frame is converted (not 62.5%
of the height), and restores the standard MIPI data-type filter range
[0x12, 0x2f] that the custom-depth path leaves at [0, 0] (which would drop all
RAW10 packets and yield a blank preview).
