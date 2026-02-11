# Copyright Open Logistics Foundation
# 
# Licensed under the Open Logistics Foundation License 1.3.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: OLFL-1.3
# 

# SYSTEM Property for successful compilation of third party deps 
# https://cmake.org/cmake/help/latest/prop_tgt/SYSTEM.html#prop_tgt:SYSTEM
cmake_minimum_required(VERSION 3.21)

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

project(libvda5050++
    VERSION 3.1.0
    DESCRIPTION "Helper library for building VDA 5050 compliant software"
    LANGUAGES CXX)

# declare CPM
include(cmake/CPM.cmake)
option(LIBVDA5050PP_VDA5050_MESSAGE_STRUCTS_DEP_LOCAL "libVDA5050++: Use vda5050_message_structs soruces next to libvda5050pp folder" OFF)

# check environment if dependencies are forced to be local
if($ENV{libvda5050pp_dev_env_local})
    message(STATUS "Force development dependencies to be included as local repositories.")
    set(LIBVDA5050PP_VDA5050_MESSAGE_STRUCTS_DEP_LOCAL ON)
endif()

if(LIBVDA5050PP_VDA5050_MESSAGE_STRUCTS_DEP_LOCAL)
    set(CPM_vda5050_message_structs_SOURCE ${PROJECT_SOURCE_DIR}/../vda5050_message_structs)
endif()

# Compiler flag function
include(CheckCXXCompilerFlag)

function(set_compiler_flag flag)
    check_cxx_compiler_flag("${flag}" flag_supported)

    if(flag_supported)
        set(LIBVDA5050PP_CMAKE_CXX_FLAGS "${LIBVDA5050PP_CMAKE_CXX_FLAGS};${flag}" PARENT_SCOPE)
    else()
        message(STATUS "CXX_FLAG ${flag} not supported")
    endif()

    unset(flag_supported CACHE)
endfunction()

# Compiler -W flags
option(LIBVDA5050PP_ENABLE_W_FLAGS "Enable all -W* flags for the libVDA5050++" ON)
if (${LIBVDA5050PP_ENABLE_W_FLAGS})
    message("Enabling -W* flags for libVDA5050++")
    set_compiler_flag("-Wall")
    set_compiler_flag("-Wextra")
    if (NOT CMAKE_VERSION VERSION_LESS "3.25")
        set_compiler_flag("-Wshadow")
        set_compiler_flag("-Wpedantic")
        set_compiler_flag("-Wconversion")
    else()
        message(WARNING "Some warnings will be suppressed. Consider upgrading to CMake version >= 3.25!")
    endif()
    set_compiler_flag("-Werror")
else()
    message("Disabling -W* flags for libVDA5050++")
    set_compiler_flag("-w")
endif()

# CI-Options
option(LIBVDA5050PP_CI_ENABLE_ALL "Enable all build options (used by CI)" OFF)

if(LIBVDA5050PP_CI_ENABLE_ALL)
    message(STATUS "Enabling libVDA5050++ CI-Build")
    set(LIBVDA5050PP_BUILD_DOCS ON)
    set(BUILD_TESTING ON)
    set(CODE_COVERAGE ON)
    set(LIBVDA5050PP_BUILD_STATIC ON)
endif()

# Debug flags
option(LIBVDA5050PP_USE_GLIBCXX_DEBUG "Use -D_GLIBCXX_DEBUG" OFF)

if(LIBVDA5050PP_USE_GLIBCXX_DEBUG)
    message(STATUS "Using directory wide _GLIBCXX_DEBUG for DEBUG config")
    add_compile_definitions(
        $<$<CONFIG:Debug>:_GLIBCXX_DEBUG>
    )
endif()

# Address Sanitizer
option(LIBVDA5050PP_ENABLE_ASAN "Enable the address sanitizer" OFF)

# Thread Sanitizer
option(LIBVDA5050PP_ENABLE_TSAN "Enable the thread sanitizer" OFF)

if(LIBVDA5050PP_ENABLE_ASAN AND LIBVDA5050PP_ENABLE_TSAN)
    message(FATAL_ERROR "Cannot enable the address and thread sanitizer")
endif()

if(LIBVDA5050PP_ENABLE_ASAN)
    message(STATUS "Enabling the address sanitizer")

    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        if (CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
            # clang with clang-cl frontend
            set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fsanitize=address")
            set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} /fsanitize=address")
        else()
            # clang with regular frontend
            set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
            set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
        endif()
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # gcc
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        # intel c/c++ compiler
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        # msvc
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fsanitize=address")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} /fsanitize=address")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        # apple clang
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
    endif()
endif()

if(LIBVDA5050PP_ENABLE_TSAN)
    message(STATUS "Enabling the thread sanitizer")

    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        if (CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
            # clang with clang-cl frontend
            set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fsanitize=thread")
            set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} /fsanitize=thread")
        else()
            # clang with regular frontend
            set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
            set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
        endif()
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # gcc
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        # intel c/c++ compiler
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        # msvc
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fsanitize=thread")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} /fsanitize=thread")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        # apple clang
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
        set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
    endif()
endif()

if(LIBVDA5050PP_ENABLE_ASAN OR LIBVDA5050PP_ENABLE_TSAN)
    message("Remember to run 'sudo sysctl -w vm.mmap_rnd_bits=28' otherwise the sanitizer might not work!")
endif()

# If this is the main project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(LIBVDA5050PP_MAIN_PROJECT ON)

    # use c++ instead of g++
    set(CMAKE_CXX_EXTENSIONS OFF)

    # include doxygen
    option(LIBVDA5050PP_BUILD_DOCS "Build libVDA5050++ documentation with doxygen and mkdocs" OFF)

    if(LIBVDA5050PP_BUILD_DOCS)
        add_subdirectory(docs)
    endif()
else()
    set(LIBVDA5050PP_MAIN_PROJECT OFF)
endif()

# Logging part
set(LIBVDA5050PP_AUX_DEFINITIONS "")
option(LIBVDA5050PP_EXPOSE_LOGGER "Make the libVDA5050++ expose access to it's loggers (exposes spdlog dependency)." OFF)
if (LIBVDA5050PP_EXPOSE_LOGGER)
    message(STATUS "Exposing spdlog loggers.")
    list(APPEND LIBVDA5050PP_AUX_DEFINITIONS "LIBVDA5050PP_EXPOSE_LOGGER")
    set(LIBVDA5050PP_INSTALL_CONFIG_EXTRA_LINES "find_package(spdlog ${LIBVDA5050PP_SPDLOG_VERSION} REQUIRED)")
endif()

# only use code coverage with clang (include here, such that child projects do not initialize
# code-cov before us)
if(BUILD_TESTING AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Include to enable code coverage for SonarQube
    include(cmake/code-coverage.cmake)
endif()

# include dependencies
include(cmake/ensure-deps.cmake)

# check platform 
include(cmake/platform.cmake)

# include libvda5050++ source code
add_subdirectory(src)

# include tests
include(CTest)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# only use code coverage with clang
if(CODE_COVERAGE AND BUILD_TESTING AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Enabling code coverage for vda5050++ and vda5050++_test")
    target_code_coverage(vda5050++)
    target_code_coverage(vda5050++_test
        ARGS
        --reporter junit --out vda5050++_test.xml # report for for gitlab ci
        --order lex # this is a rather ugly workaround, that prevents static initialization
                    # bugs of the instance in the test-cases
    )
endif()

# install
option(LIBVDA5050PP_INSTALL "Generate libVDA5050++ install targets." ${LIBVDA5050PP_MAIN_PROJECT})
option(LIBVDA5050PP_CLEAN_INSTALL "Cleanup unused installed libVDA5050++ files. (May interfere with (not found) exsting installations of dependencies used by libVDA5050++. Check cmake/PostInstall.sh)" OFF)
option(LIBVDA5050PP_BUILD_DEB "Build a .deb package (UNIX only)." OFF)

if(LIBVDA5050PP_INSTALL)
    include(cmake/install.cmake)

    if(LIBVDA5050PP_BUILD_DEB)
        include(cmake/deb.cmake)
    endif()

    include(cmake/enable-clean-install.cmake)
endif()

# examples
option(LIBVDA5050PP_EXAMPLES "Generate libVDA5050++ exampes targets." OFF})

if(LIBVDA5050PP_EXAMPLES)
    message(STATUS "Generating libVDA5050++ example targets.")
    add_subdirectory(examples)
endif()
