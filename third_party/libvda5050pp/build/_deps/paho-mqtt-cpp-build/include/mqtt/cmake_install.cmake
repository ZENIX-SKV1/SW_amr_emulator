# Install script for directory: /home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mqtt" TYPE FILE FILES
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/async_client.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/buffer_ref.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/buffer_view.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/callback.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/client.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/connect_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/create_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/delivery_token.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/disconnect_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/event.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/exception.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/export.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/iaction_listener.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/iasync_client.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/iclient_persistence.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/message.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/platform.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/properties.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/reason_code.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/response_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/server_response.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/ssl_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/string_collection.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/subscribe_options.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/thread_queue.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/token.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/topic_matcher.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/topic.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/types.h"
    "/home/zenix/ros2_ws/src/amr_emulator/third_party/libvda5050pp/build/_deps/paho-mqtt-cpp-src/include/mqtt/will_options.h"
    )
endif()

