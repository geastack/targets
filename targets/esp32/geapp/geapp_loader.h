// SPDX-License-Identifier: GPL-3.0-only
// Loads a .geapp program: a position-independent RISC-V shared object built
// without the runtime, linked on the device against the firmware's export
// table (geapp_exports.h).
#pragma once

#include <cstddef>
#include <cstdint>

namespace gea::platform::geapp {

struct Module;

// Reads the shared object at `path` into executable PSRAM, binds its imports
// to the firmware's exports, registers its unwind tables and runs its static
// constructors. On failure returns nullptr and writes the reason to `error`.
Module *load(const char *path, char *error, std::size_t errorSize);
// load() in two halves. map() reads, relocates and publishes the image and
// registers its unwind tables; it touches no runtime state, so it may run on a
// task of its own while the current program keeps drawing. construct() runs
// the static constructors -- program code -- and must run on the runtime's
// task. A mapped module that is never constructed is freed with discard().
Module *map(const char *path, char *error, std::size_t errorSize);
// map() of the `length` bytes at `offset` in `path`: a program entry inside a
// .geapp package, read in place.
Module *map(const char *path, std::uint32_t offset, std::uint32_t length, char *error, std::size_t errorSize);
void construct(Module *module);
void discard(Module *module);

// The firmware's symbol a module's import would bind to, for diagnostics.
std::uintptr_t firmwareSymbol(const char *name);

// The address of a symbol the module itself defines, or nullptr.
void *symbol(const Module *module, const char *name);

// Runs the module's static destructors. Its code and data stay mapped, so
// whatever still holds one of its objects can let go of it before release().
void finalize(Module *module);

// Deregisters the module's unwind tables and frees its image. Nothing may
// still point into the module afterwards.
void release(Module *module);

// What the last unload took back that the program left behind: the proof a
// leaking app cost Home nothing (GEADEV APPS RELEASED prints it).
struct ReleaseReport {
	unsigned tasks = 0;
	unsigned isrs = 0;
	unsigned espTimers = 0;
	unsigned rtosTimers = 0;
	unsigned queues = 0;
	unsigned files = 0;  // FILEs, directories and descriptors (sockets too)
	unsigned blocks = 0;
	unsigned long long blockBytes = 0;
	bool recordOverflowed = false;
	unsigned releases = 0;  // how many programs have been released since boot
};
const ReleaseReport &lastRelease();

// finalize() then release().
void unload(Module *module);

bool contains(const Module *module, const void *address);
std::uintptr_t base(const Module *module);
std::size_t imageSize(const Module *module);
// What a code address on the instruction bus is above the same byte on the
// data bus (0 where code runs from its data address).
std::uintptr_t codeOffset();

}  // namespace gea::platform::geapp
