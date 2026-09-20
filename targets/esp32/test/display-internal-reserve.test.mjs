// The display's internal-RAM reserve is a real call on the platform Memory API,
// never a weak hook: a weak *undefined* reference does not pull the defining
// member out of a static archive, so memory.cpp.obj stayed in libmain.a, the
// pointer linked to null, the `if (hook)` guard skipped the reserve silently,
// and the board came up with a blank panel. No log line, no link error.
//
// @geastack/core guards the framework half (the API is a method, the runtime
// takes and releases the reserve around Display::init()). This is the esp32
// half: the target actually implements it, with DMA-capable internal memory.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const esp32Memory = readFileSync(new URL('../services/memory.cpp', import.meta.url), 'utf8')

// DMA capability, not plain internal: descriptors and the transaction pool are
// what an SPI/LCD driver cannot allocate out of non-DMA internal RAM.
assert.match(esp32Memory, /void \*Memory::reserveInternalDma\(std::size_t bytes\)/)
assert.match(esp32Memory, /heap_caps_malloc\(bytes, MALLOC_CAP_INTERNAL \| MALLOC_CAP_DMA\)/)
assert.match(esp32Memory, /void Memory::releaseInternalDma\(void \*reserve\)/)
assert.doesNotMatch(esp32Memory, /extern "C" void \*gea_platform_reserve_internal_dma/)
