#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libvda5050++::vda5050++" for configuration "Release"
set_property(TARGET libvda5050++::vda5050++ APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libvda5050++::vda5050++ PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvda5050++.so.3.1.0"
  IMPORTED_SONAME_RELEASE "libvda5050++.so.0"
  )

list(APPEND _cmake_import_check_targets libvda5050++::vda5050++ )
list(APPEND _cmake_import_check_files_for_libvda5050++::vda5050++ "${_IMPORT_PREFIX}/lib/libvda5050++.so.3.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
