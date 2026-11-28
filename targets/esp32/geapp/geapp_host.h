// SPDX-License-Identifier: GPL-3.0-only
// The firmware side of installable programs: the GeaOS host that loads an
// installed app's program into this firmware (gea::geaos::AppHost), and the
// program::Host the runtime switches programs through.
//
// Apps are found by the GeaOS registry (geaos.h) on the volumes the board
// registers -- its NAND, its flash data partition, its SD card -- as a folder
//   <volume>/apps/<id>/manifest.json + program/esp32-riscv32 (or program.so)
// or a single package <volume>/apps/<id>.geapp.
#pragma once

// The largest program image Home loads, in bytes (0: no limit). The CLI sets
// it from Home's gea.targets.esp32.geapp.maxProgramBytes.
#ifndef GEA_GEAPP_MAX_PROGRAM_BYTES
#define GEA_GEAPP_MAX_PROGRAM_BYTES 0
#endif

#include <string>

#include "geaos.h"
#include "sdkconfig.h"

namespace gea::platform::geapp {

// The program entry this host runs (cli/src/esp32/geapp.mjs programTargetKey).
#if CONFIG_IDF_TARGET_ESP32S3
inline constexpr char kTargetKey[] = "esp32-xtensa-s3";
#else
inline constexpr char kTargetKey[] = "esp32-riscv32";
#endif

// The ABI this firmware's programs must be built for (the CLI writes the same
// string into every manifest it builds against this firmware).
const char *firmwareAbi();

// Where the running program's image is loaded (0 with none): what a crash
// or heap-trace address inside it is symbolized against.
unsigned long loadedProgramBase();

// Registers the loader with the runtime (program::setHost) and with GeaOS
// (Launcher::setHost), and starts the button that returns a running program
// to the firmware's own. Idempotent.
void installHost();

// Mounts the board's storage if it is not yet, polls every volume source
// and rescans every volume. True if the set of apps changed.
bool rescan();

// Mounts the boot flash's "apps" FAT partition at /apps and registers it as
// the "flash" volume, when the partition table has one. installHost() calls
// it; true once mounted.
bool mountFlashVolume();
// Why the flash volume is or is not mounted ("mounted apps at /apps: ...",
// "no FAT partition labelled apps", "mount of apps failed: ...").
const char *flashVolumeStatus();

// Volume monitoring (geapp_volumes.cpp). A board registers each place apps
// can come and go -- an SD slot, a USB drive -- as a VolumeSource; fixed
// storage it mounts itself adds with Launcher::addVolume. installHost()
// starts the task that polls every 2 s; a card-detect interrupt wakes it
// early.
void addVolumeSource(gea::geaos::VolumeSource *source);
bool pollVolumes();
void wakeVolumeMonitor();
void wakeVolumeMonitorFromIsr();
void startVolumeMonitor();

// Registers the remote-install routes (GET /apps, POST /apps/install,
// GET /apps/install/status, DELETE /apps/<id>) on an esp_http_server
// handle. Returns how many registered. (geapp_http.cpp)
int registerHttpRoutes(void *httpServer);

// A file path for the app's icon ("" for none). A package app's icon is
// copied out once to <volume>/apps/.icons/ so an <img> can load it.
std::string iconPath(const gea::geaos::InstalledApp &app);

}  // namespace gea::platform::geapp
