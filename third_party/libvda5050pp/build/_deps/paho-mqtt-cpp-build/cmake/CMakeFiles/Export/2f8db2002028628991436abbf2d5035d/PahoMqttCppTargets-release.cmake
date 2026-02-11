#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PahoMqttCpp::paho-mqtt3a" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqtt3a APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqtt3a PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqtt3a.so.1.3.14"
  IMPORTED_SONAME_RELEASE "libpaho-mqtt3a.so.1"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqtt3a )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqtt3a "${_IMPORT_PREFIX}/lib/libpaho-mqtt3a.so.1.3.14" )

# Import target "PahoMqttCpp::paho-mqtt3as" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqtt3as APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqtt3as PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqtt3as.so.1.3.14"
  IMPORTED_SONAME_RELEASE "libpaho-mqtt3as.so.1"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqtt3as )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqtt3as "${_IMPORT_PREFIX}/lib/libpaho-mqtt3as.so.1.3.14" )

# Import target "PahoMqttCpp::paho-mqtt3a-static" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqtt3a-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqtt3a-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqtt3a.a"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqtt3a-static )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqtt3a-static "${_IMPORT_PREFIX}/lib/libpaho-mqtt3a.a" )

# Import target "PahoMqttCpp::paho-mqtt3as-static" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqtt3as-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqtt3as-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqtt3as.a"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqtt3as-static )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqtt3as-static "${_IMPORT_PREFIX}/lib/libpaho-mqtt3as.a" )

# Import target "PahoMqttCpp::paho-mqttpp3-shared" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqttpp3-shared APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqttpp3-shared PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqttpp3.so.1.5.1"
  IMPORTED_SONAME_RELEASE "libpaho-mqttpp3.so.1"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqttpp3-shared )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqttpp3-shared "${_IMPORT_PREFIX}/lib/libpaho-mqttpp3.so.1.5.1" )

# Import target "PahoMqttCpp::paho-mqttpp3-static" for configuration "Release"
set_property(TARGET PahoMqttCpp::paho-mqttpp3-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(PahoMqttCpp::paho-mqttpp3-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpaho-mqttpp3.a"
  )

list(APPEND _cmake_import_check_targets PahoMqttCpp::paho-mqttpp3-static )
list(APPEND _cmake_import_check_files_for_PahoMqttCpp::paho-mqttpp3-static "${_IMPORT_PREFIX}/lib/libpaho-mqttpp3.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
