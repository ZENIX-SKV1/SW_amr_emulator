cmake_minimum_required(VERSION 3.10)
project(amr_emulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(
    src 
    src/domain/module/amr
    src/domain/module/localizer
    src/domain/module/motor_controller
    src/domain/module/navigator
    src/domain/module/vcu
    src/domain/common
    src/domain/models/acceleration
    src/domain/models/dead_reckoning
    src/domain/models/battery
    src/domain/protocols
    src/app
    src/infrastructure 
    src/presentation 
    src/utils 
    third_party/yaml-cpp/include
    third_party/libvda5050pp/include/public/vda5050++
    third_party/libvda5050pp/include/private
    third_party/rapidjson/include
    )
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/libvda5050pp)
add_subdirectory(third_party/driving_viewer)
file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(amr_emulator ${SOURCES})
# target_link_libraries(amr_emulator yaml-cpp ws2_32)
target_link_libraries(amr_emulator yaml-cpp vda5050++)
add_subdirectory(test)
