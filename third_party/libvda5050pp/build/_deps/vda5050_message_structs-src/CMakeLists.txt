# Copyright Open Logistics Foundation
# 
# Licensed under the Open Logistics Foundation License 1.3.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: OLFL-1.3
# 

cmake_minimum_required(VERSION 3.16) # Adds instrumentation to all targets

# Do not check system dependencies for changes, when rebuilding.
# Ninja build scripts always flag them as dirty, so each time the project is built
# every file is (wrongly) forcefully rebuilt.
if(CMAKE_GENERATOR MATCHES "Ninja")
  file(
    WRITE "${CMAKE_BINARY_DIR}/GNUMakeRulesOverwrite.cmake"
    "STRING(REPLACE \"-MD\" \"-MMD\" CMAKE_DEPFILE_FLAGS_C \"\${CMAKE_DEPFILE_FLAGS_C}\")\n"
    "STRING(REPLACE \"-MD\" \"-MMD\" CMAKE_DEPFILE_FLAGS_CXX \"\${CMAKE_DEPFILE_FLAGS_CXX}\")\n"
  )
  set(CMAKE_USER_MAKE_RULES_OVERRIDE "${CMAKE_BINARY_DIR}/GNUMakeRulesOverwrite.cmake" CACHE INTERNAL "")
endif()

project(vda5050_message_structs
  VERSION 2.1.0.3
  DESCRIPTION "This Project contains C++ structs representing the all VDA5050 message objects"
  LANGUAGES CXX)

# declare CPM
include(cmake/CPM.cmake)

set(LIBVDA5050PP_NLOHMANN_JSON_VERSION "3.11.3" CACHE STRING "Overwrite nlohmann_json version")
find_package(nlohmann_json ${LIBVDA5050PP_NLOHMANN_JSON_VERSION} QUIET)

if(nlohmann_json_FOUND)
  message(STATUS "Found nlohman_json installation with version ${nlohmann_json_VERSION}")
else()
  set(JSON_Install ON)
  CPMAddPackage(
    NAME nlohmann_json
    VERSION ${LIBVDA5050PP_NLOHMANN_JSON_VERSION}
    GITHUB_REPOSITORY nlohmann/json
    OPTIONS
    "JSON_BuildTests OFF"
  )
  set(nlohmann_json_VERSION ${LIBVDA5050PP_NLOHMANN_JSON_VERSION})
endif()

# Check for platform compatibility
include(cmake/platform.cmake)

set(VDA5050_MESSAGE_STRUCTS_SOURCES
  src/json.cpp
)

add_library(vda5050_message_structs-static STATIC ${VDA5050_MESSAGE_STRUCTS_SOURCES})
target_include_directories(vda5050_message_structs-static PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(vda5050_message_structs-static PUBLIC cxx_std_17)
target_link_libraries(vda5050_message_structs-static PUBLIC nlohmann_json::nlohmann_json)
target_compile_definitions(vda5050_message_structs-static PRIVATE ${VDA5050_MESSAGE_STRUCTS_PLATFORM_DEFINITIONS})

add_library(vda5050_message_structs SHARED ${VDA5050_MESSAGE_STRUCTS_SOURCES})
set_target_properties(vda5050_message_structs PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 0
)
target_include_directories(vda5050_message_structs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(vda5050_message_structs PUBLIC cxx_std_17)
target_link_libraries(vda5050_message_structs PUBLIC nlohmann_json::nlohmann_json)
target_compile_definitions(vda5050_message_structs PRIVATE ${VDA5050_MESSAGE_STRUCTS_PLATFORM_DEFINITIONS})

# include tests
include(CTest)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

option(VDA5050_MESSAGE_STRUCTS_COVERAGE "Enable llvm-cov targets for vda5050_message_structs" OFF)
if (VDA5050_MESSAGE_STRUCTS_COVERAGE)
  message(STATUS "Enabling code coverage for vda5050_message_structs and vda5050_message_structs_test")
  set(CODE_COVERAGE ON)
  include(cmake/code-coverage.cmake)
  target_code_coverage(vda5050_message_structs-static)
  target_code_coverage(vda5050_message_structs_test
      ARGS
      --reporter junit --out vda5050_message_structs_test.xml
  )
endif()

# # Install part ################################################################
# gather install targets
install(
  TARGETS
  vda5050_message_structs
  vda5050_message_structs-static
  EXPORT vda5050_message_structsTargets
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/vda5050 DESTINATION include)

# make targets available
install(EXPORT vda5050_message_structsTargets
  FILE vda5050_message_structsTargets.cmake
  DESTINATION lib/cmake/vda5050_message_structs
)

# generate the config file that is includes the exports
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/vda5050_message_structsConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/vda5050_message_structs"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# generate the version matching file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/vda5050_message_structsConfigVersion.cmake"
  VERSION "${vda5050_message_structs_VERSION_MAJOR}.${vda5050_message_structs_VERSION_MINOR}"
  COMPATIBILITY SameMajorVersion
)

# install helper files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/vda5050_message_structsConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/vda5050_message_structsConfigVersion.cmake
  DESTINATION lib/cmake/vda5050_message_structs
)

# make targets available, if this is a subdirectory
export(EXPORT vda5050_message_structsTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/vda5050_message_structsTargets.cmake"
)

# ###############################################################################
