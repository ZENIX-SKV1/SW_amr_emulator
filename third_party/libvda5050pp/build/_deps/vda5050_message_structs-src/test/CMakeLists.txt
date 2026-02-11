# Copyright Open Logistics Foundation
# 
# Licensed under the Open Logistics Foundation License 1.3.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: OLFL-1.3
# 

set(LIBVDA5050PP_CATCH2_VERSION "3.4.0" CACHE STRING "Overwrite catch2 version")
find_package(Catch2 ${LIBVDA5050PP_CATCH2_VERSION} QUIET)

if(Catch2_FOUND)
  message(STATUS "Found Catch2 installation.")
else()
  CPMAddPackage(
    NAME Catch2
    GITHUB_REPOSITORY catchorg/Catch2
    VERSION ${LIBVDA5050PP_CATCH2_VERSION}
  )
endif()

if(Catch2_SOURCE_DIR)
  list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
endif()

include(Catch)


add_executable(vda5050_message_structs_test
  ${PROJECT_SOURCE_DIR}/test/src/test.cpp
)
target_include_directories(vda5050_message_structs_test PRIVATE
  ${PROJECT_SOURCE_DIR}/test/include
)
target_link_libraries(vda5050_message_structs_test PRIVATE
  vda5050_message_structs-static
  Catch2::Catch2WithMain
)

# Let CTest discover the Catch2 test cases
catch_discover_tests(vda5050_message_structs_test)