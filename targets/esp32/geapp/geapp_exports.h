// SPDX-License-Identifier: GPL-3.0-only
// The firmware's export table: every global symbol a .geapp may bind to,
// keyed by the 64-bit FNV-1a hash of its name and sorted by that hash.
//
// The table is a reserved, fixed-size array in the firmware image. After the
// firmware links, the build fills it from the linked ELF's own symbol table
// (cli/src/esp32/geapp/home-exports.mjs) before the ELF becomes the flashable
// image, so every address in it is exact. A firmware that was never filled has
// count 0 and loads no program.
#pragma once

#include <cstddef>
#include <cstdint>

namespace gea::platform::geapp {

inline constexpr char kExportsMagic[8] = {'G', 'E', 'A', 'X', 'P', 'O', 'R', 'T'};
inline constexpr std::uint32_t kExportsVersion = 1;

struct ExportsHeader {
	char magic[8];
	std::uint32_t version;
	std::uint32_t count;
	std::uint32_t capacity;
	std::uint32_t reserved;
};

struct ExportEntry {
	std::uint64_t hash;
	std::uint32_t address;
	std::uint32_t reserved;
};

constexpr std::uint64_t exportHash(const char *name)
{
	std::uint64_t hash = 0xcbf29ce484222325ull;
	for (; *name; ++name) {
		hash ^= static_cast<unsigned char>(*name);
		hash *= 0x100000001b3ull;
	}
	return hash;
}

// The firmware address of `name`, or 0 when the firmware does not export it.
std::uintptr_t findExport(const char *name);
std::uint32_t exportCount();

}  // namespace gea::platform::geapp
