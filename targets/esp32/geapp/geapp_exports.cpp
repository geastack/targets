// SPDX-License-Identifier: GPL-3.0-only
#include "geapp_exports.h"

#ifndef GEA_GEAPP_EXPORTS_CAPACITY
#define GEA_GEAPP_EXPORTS_CAPACITY 32768
#endif

namespace gea::platform::geapp {

namespace {

struct ExportsTable {
	ExportsHeader header;
	ExportEntry entries[GEA_GEAPP_EXPORTS_CAPACITY];
};

}  // namespace

}  // namespace gea::platform::geapp

// Filled after link by cli/src/esp32/geapp/home-exports.mjs, which finds it by
// this symbol name and checks the magic and capacity before writing.
extern "C" __attribute__((used, aligned(16), section(".rodata.gea_home_exports")))
const gea::platform::geapp::ExportsTable gea_home_exports = {
	{{'G', 'E', 'A', 'X', 'P', 'O', 'R', 'T'}, gea::platform::geapp::kExportsVersion, 0, GEA_GEAPP_EXPORTS_CAPACITY, 0},
	{},
};

namespace gea::platform::geapp {

namespace {

// The compiler sees the table's initializer (count 0); the build rewrites it
// after link. Read it through an opaque pointer so nothing folds.
const ExportsTable *table()
{
	const ExportsTable *result = &gea_home_exports;
	asm volatile("" : "+r"(result));
	return result;
}

}  // namespace

std::uint32_t exportCount()
{
	const ExportsTable *exports = table();
	for (std::size_t i = 0; i < sizeof(kExportsMagic); ++i)
		if (exports->header.magic[i] != kExportsMagic[i]) return 0;
	if (exports->header.version != kExportsVersion) return 0;
	return exports->header.count <= exports->header.capacity ? exports->header.count : 0;
}

std::uintptr_t findExport(const char *name)
{
	const std::uint32_t count = exportCount();
	if (count == 0) return 0;
	const ExportEntry *entries = table()->entries;
	const std::uint64_t hash = exportHash(name);
	std::uint32_t low = 0;
	std::uint32_t high = count;
	while (low < high) {
		const std::uint32_t middle = low + (high - low) / 2;
		if (entries[middle].hash < hash) low = middle + 1;
		else high = middle;
	}
	if (low < count && entries[low].hash == hash) return entries[low].address;
	return 0;
}

}  // namespace gea::platform::geapp
