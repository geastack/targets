// Compile the real chunk policy in both panel modes. CO5300 and RM69080
// fallbacks must preserve even-row windows, including odd requests and tails.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'

const source = readFileSync(new URL('../display.cpp', import.meta.url), 'utf8')
const minimum = source.match(/constexpr int kFlushChunkMin = \d+;/)[0]
function method(name) {
  const body = source.match(new RegExp(`    int ${name}\\(int (\\w+)\\) const\\n    \\{([\\s\\S]*?)\\n    \\}`))
  assert.ok(body, `missing ${name}`)
  return `constexpr int ${name}(int ${body[1]}) {${body[2]}\n}`
}
const rotated = source.match(/const int rotatedChunkRows = \[&\] \{([\s\S]*?)\n      \}\(\);/)[1]
const packed = source.match(/const int packedChunkRows = \[&\] \{([\s\S]*?)\n      \}\(\);/)[1]
assert.match(source, /#if GEA_EMBEDDED_RM690B0_PANEL[\s\S]*?panel\(\)\.drawBitmap\(\s*colStart, physicalRow, colEnd \+ 1, physicalRow \+ rows, buffer\)/)
for (const panel of [0, 1]) {
  const program = `
#define GEA_EMBEDDED_RM690B0_PANEL ${panel}
constexpr int kFlushChunkDefault = 16;
constexpr int kFlushChunkLimit = 16;
${minimum}
${method('normalizeFlushChunkRows')}
${method('nextSmallerChunk')}
constexpr int rotatedRows(int capacity, int columns) {
  int flushBufferCapacity_ = capacity;
  int colCount = columns;
  int flushChunkRows_ = kFlushChunkMin;
  return [&] {${rotated}
}();
}
constexpr int packedRows(int capacity, int columns, int request) {
  int flushBufferCapacity_ = capacity;
  int width = columns;
  int requestedFlushChunkRows_ = request;
  return [&] {${packed}
}();
}
constexpr bool packedWindows() {
  for (int columns = 2; columns <= 450; columns += 2) {
    for (int request = 2; request <= 64; ++request) {
      int rows = packedRows(900, columns, request);
      if (rows < 2 || (rows & 1) || rows * columns > 900) return false;
      if (rows > normalizeFlushChunkRows(request)) return false;
    }
  }
  return true;
}
static_assert(packedWindows(), "portrait chunks broke capacity, requested limit, or alignment");
static_assert(packedRows(900, 450, 16) == 2, "full-width fallback must stay unchanged");
static_assert(packedRows(900, 50, 16) == 16, "narrow rect must use available staging");
constexpr bool rotatedWindows() {
  for (int columns = 2; columns <= 450; columns += 2) {
    int rows = rotatedRows(450 * kFlushChunkMin, columns);
    if (rows < kFlushChunkMin || rows * columns > 450 * kFlushChunkMin) return false;
    if (GEA_EMBEDDED_RM690B0_PANEL && (rows & 1)) return false;
  }
  return true;
}
static_assert(rotatedWindows(), "narrow rotated windows broke row alignment or staging capacity");
constexpr bool validWindows() {
  for (int request = -1; request <= 64; ++request) {
    int previous = 65;
    for (int rows = normalizeFlushChunkRows(request); rows; rows = nextSmallerChunk(rows)) {
      if (rows >= previous || rows < kFlushChunkMin || rows > 16) return false;
      previous = rows;
      for (int y = 0; y < 600; y += rows) {
        int count = y + rows > 600 ? 600 - y : rows;
        if ((y & 1) || (count & 1)) return false;
      }
    }
  }
  return true;
}
static_assert(validWindows(), "fallback split broke panel row alignment");
static_assert(normalizeFlushChunkRows(1) == 2);
static_assert(normalizeFlushChunkRows(3) == 2);
static_assert(nextSmallerChunk(4) == 2);
static_assert(nextSmallerChunk(2) == 0);
`
  const result = spawnSync(process.env.CXX || 'c++', ['-std=c++17', '-x', 'c++', '-fsyntax-only', '-'], {
    input: program, encoding: 'utf8',
  })
  assert.equal(result.status, 0, result.stderr || String(result.error))
}
console.log('RM69080 and CO5300 chunk alignment passed')
