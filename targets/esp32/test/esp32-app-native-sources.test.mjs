import assert from 'node:assert/strict'
import { existsSync, readFileSync, readdirSync } from 'node:fs'
import { dirname, join, resolve } from 'node:path'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const targetsRoot = join(repoRoot, 'targets')
const frameworkCmake = readFileSync(join(targetsRoot, 'esp32', 'gea_framework.cmake'), 'utf8')
const frameworkComponentCmake = readFileSync(
  join(targetsRoot, 'esp32', 'components', 'gea_framework', 'CMakeLists.txt'),
  'utf8'
)




assert.match(
  frameworkCmake,
  /_gea_option STREQUAL "-iquote"/,
  'the stable framework component should not inherit generated app include paths'
)
assert.doesNotMatch(
  frameworkCmake,
  /add_dependencies\([^\n]*gea_framework[^\n]*geatsc_build|gea_framework_add_dependency\(geatsc_build\)/,
  'stable framework compilation should run independently of app code generation'
)
assert.doesNotMatch(
  frameworkComponentCmake,
  /CMAKE_BINARY_DIR/,
  'the stable component should not put an app-specific build directory in its compile command'
)
for (const appSensitiveDefinition of [
  'GEA_EMBEDDED_APP_USES_BLE',
  'GEA_EMBEDDED_APP_USES_AUDIO',
  'GEA_EMBEDDED_BLE_DISABLED',
  'GEA_EMBEDDED_AUDIO_UNUSED',
  'GEA_EMBEDDED_ENABLE_HTTPS',
  'GEA_EMBEDDED_WIFI_DISABLED',
  'GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH',
  'GEA_EMBEDDED_SUBTREE_REVEAL_CHECK',
  'GEA_EMBEDDED_SKIP_POSITION_INLINE_RECORD',
]) {
  assert.ok(
    frameworkCmake.includes(appSensitiveDefinition),
    `the stable framework should exclude app-sensitive definition ${appSensitiveDefinition}`
  )
}
assert.match(
  frameworkCmake,
  /\)\(=\.\*\)\?\$"\)/,
  'app-sensitive definition filtering should handle both bare and valued definitions'
)
// Only IDF PROJECTS get checked here. A directory without a CMake project is a
// board DEFINITION (board.json + board.mjs, no CMakeLists.txt) whose project is
// generated at build time, so it has no checked-in sdkconfig.defaults and never
// should. Presence of a top-level CMakeLists.txt is what makes a directory a
// project, so filter on that rather than on the esp32- name prefix alone.
const esp32TargetDirectories = readdirSync(targetsRoot, { withFileTypes: true })
  .filter((entry) => entry.isDirectory() && entry.name.startsWith('esp32-'))
  .filter((entry) => existsSync(join(targetsRoot, entry.name, 'CMakeLists.txt')))

assert.ok(esp32TargetDirectories.length > 0, 'expected at least one ESP32 IDF project directory')

for (const targetDirectory of esp32TargetDirectories) {
  const projectCmakePath = join(targetsRoot, targetDirectory.name, 'CMakeLists.txt')
  assert.equal(
    existsSync(join(targetsRoot, targetDirectory.name, 'sdkconfig.defaults')),
    true,
    `${targetDirectory.name} should provide immutable defaults for its build-local sdkconfig files`
  )
  assert.match(
    readFileSync(projectCmakePath, 'utf8'),
    /idf_build_set_property\(MINIMAL_BUILD ON\)/,
    `${projectCmakePath} should build only main and its transitive IDF dependencies`
  )
  // A board may list several component directories (its own alongside the shared
  // one), in which case set() spans multiple lines -- so match the whole call
  // rather than requiring the shared path on the same line as the variable.
  const projectCmake = readFileSync(projectCmakePath, 'utf8')
  const extraComponentDirs = projectCmake.match(/set\(EXTRA_COMPONENT_DIRS[\s\S]*?\)/)?.[0] ?? ''
  assert.ok(extraComponentDirs, `${projectCmakePath} should set EXTRA_COMPONENT_DIRS`)
  assert.match(
    extraComponentDirs,
    /esp32\/components/,
    `${projectCmakePath} should discover the shared stable framework component`
  )
}

// The CLI hands every target the same GEA_EMBEDDED_APP_META list
// (root;entry;runtime;native sources...); the shared framework cmake parses it
// once so no board spawns node or re-derives the app layout.
assert.match(
  frameworkCmake,
  /list\(SUBLIST GEA_EMBEDDED_APP_META_LIST 3 -1 GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS\)/,
  'gea_framework.cmake should parse app-owned native sources from GEA_EMBEDDED_APP_META'
)
// Element 0 IS the app directory, absolute, and no consumer may rejoin it onto
// anything.
assert.match(frameworkCmake, /list\(GET GEA_EMBEDDED_APP_META_LIST 0 GEA_EMBEDDED_APP_DIR\)/)
assert.match(frameworkCmake, /if\(NOT IS_ABSOLUTE "\$\{GEA_EMBEDDED_APP_DIR\}"\)/)
assert.doesNotMatch(frameworkCmake, /GEA_APPS_ROOT\}\//, 'no consumer may rejoin the app directory onto a project root')
assert.doesNotMatch(frameworkCmake, /gea-embedded\.mjs|execute_process\([^)]*inspect/, 'the framework must not spawn the retired inspect tool')

const esp32TargetCMakeFiles = esp32TargetDirectories
  .map((entry) => join(targetsRoot, entry.name, 'main', 'CMakeLists.txt'))
  .filter((file) => existsSync(file))

assert.ok(esp32TargetCMakeFiles.length > 0, 'expected at least one ESP32 target CMake file')

for (const cmakePath of esp32TargetCMakeFiles) {
  const targetCmake = readFileSync(cmakePath, 'utf8')
  const sharedTargetInclude = targetCmake.match(
    /include\("\$\{CMAKE_CURRENT_LIST_DIR\}\/([^"$]+\/main\/CMakeLists\.txt)"\)/
  )
  const sharedTargetPath = sharedTargetInclude ? resolve(dirname(cmakePath), sharedTargetInclude[1]) : null
  const cmake = sharedTargetPath && existsSync(sharedTargetPath)
    ? `${targetCmake}\n${readFileSync(sharedTargetPath, 'utf8')}`
    : targetCmake
  assert.match(
    cmake,
    /foreach\(_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL IN LISTS GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS\)/,
    `${cmakePath} should resolve native source paths relative to the app root`
  )
  assert.match(
    cmake,
    /set\(APP_GENERATED_SOURCES[\s\S]*\$\{GEA_EMBEDDED_APP_NATIVE_SOURCES\}/,
    `${cmakePath} should compile app-owned native sources with the active app`
  )
  assert.match(
    cmake,
    /\$\{GEA_FW_APP_SENSITIVE_SOURCES\}/,
    `${cmakePath} should leave only app-sensitive framework sources in main`
  )
  assert.match(
    cmake,
    /REQUIRES[\s\S]*gea_framework/,
    `${cmakePath} should link the stable framework IDF component`
  )
  if (/add_custom_target\(geatsc_build ALL/.test(cmake)) {
    assert.match(
      cmake,
      /add_dependencies\(geatsc_active_archive geatsc_build\)/,
      `${cmakePath} should serialize the active generated archive after its geatsc producer`
    )
  }

  if (cmake.includes('function(_gea_find_sources')) {
    for (const transientDirectory of ['.scratch', '.test-tmp', 'generated-output']) {
      assert.ok(
        cmake.includes(`-name ${transientDirectory}`),
        `${cmakePath} should not watch compiler transient directory ${transientDirectory}`
      )
    }
    assert.ok(
      /_gea_find_sources\(_GEA_UPSTREAM_GEA_SRCS "\$\{GEA_COMPILER\}" PRUNE_VENDOR\)/.test(cmake) ||
        /-name vendor[\s\S]*-prune/.test(cmake),
      `${cmakePath} should not watch compiler conformance vendor trees`
    )
  }
}
