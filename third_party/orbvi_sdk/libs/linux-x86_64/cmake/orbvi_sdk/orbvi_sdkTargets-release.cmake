#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "orbvi_sdk::shared" for configuration "Release"
set_property(TARGET orbvi_sdk::shared APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(orbvi_sdk::shared PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liborbvi_sdk.so.0.1.0"
  IMPORTED_SONAME_RELEASE "liborbvi_sdk.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS orbvi_sdk::shared )
list(APPEND _IMPORT_CHECK_FILES_FOR_orbvi_sdk::shared "${_IMPORT_PREFIX}/lib/liborbvi_sdk.so.0.1.0" )

# Import target "orbvi_sdk::static" for configuration "Release"
set_property(TARGET orbvi_sdk::static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(orbvi_sdk::static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liborbvi_sdk.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS orbvi_sdk::static )
list(APPEND _IMPORT_CHECK_FILES_FOR_orbvi_sdk::static "${_IMPORT_PREFIX}/lib/liborbvi_sdk.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
