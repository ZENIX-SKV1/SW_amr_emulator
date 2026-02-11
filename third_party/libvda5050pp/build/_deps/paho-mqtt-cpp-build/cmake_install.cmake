# Install script for directory: /home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3a.so.1.3.14"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3a.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3a.so.1.3.14"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3a.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3a.so.1.3.14"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3a.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3a.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3as.so.1.3.14"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3as.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3as.so.1.3.14"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3as.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3as.so.1.3.14"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpaho-mqtt3as.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3as.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3a.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/src/libpaho-mqtt3as.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/generated/include/")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/externals/paho-mqtt-c/cmake_install.cmake")
  include("/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/include/mqtt/cmake_install.cmake")
  include("/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/src/cmake_install.cmake")
  include("/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-build/cmake/cmake_install.cmake")

endif()

