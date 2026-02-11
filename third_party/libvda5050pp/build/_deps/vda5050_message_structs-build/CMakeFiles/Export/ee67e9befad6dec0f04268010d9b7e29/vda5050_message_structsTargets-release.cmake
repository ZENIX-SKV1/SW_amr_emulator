#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vda5050_message_structs" for configuration "Release"
set_property(TARGET vda5050_message_structs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vda5050_message_structs PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvda5050_message_structs.so.2.1.0.3"
  IMPORTED_SONAME_RELEASE "libvda5050_message_structs.so.0"
  )

list(APPEND _cmake_import_check_targets vda5050_message_structs )
list(APPEND _cmake_import_check_files_for_vda5050_message_structs "${_IMPORT_PREFIX}/lib/libvda5050_message_structs.so.2.1.0.3" )

# Import target "vda5050_message_structs-static" for configuration "Release"
set_property(TARGET vda5050_message_structs-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vda5050_message_structs-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvda5050_message_structs-static.a"
  )

list(APPEND _cmake_import_check_targets vda5050_message_structs-static )
list(APPEND _cmake_import_check_files_for_vda5050_message_structs-static "${_IMPORT_PREFIX}/lib/libvda5050_message_structs-static.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
