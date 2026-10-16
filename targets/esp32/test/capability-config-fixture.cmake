set(GEA_APP_CAPABILITY_ANALYSIS_VERSION 1)
foreach(capability AUDIO CAMERA AEC)
  set(GEA_EMBEDDED_CAPABILITY_${capability} "${GEA_TEST_${capability}}" CACHE BOOL "Analyzed app capability" FORCE)
endforeach()
