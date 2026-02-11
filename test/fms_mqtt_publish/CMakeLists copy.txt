# cmake_minimum_required(VERSION 3.10)
# project(TestFmsVda5050Client CXX)
# find_package(PahoMqttCpp REQUIRED)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/json)
# add_executable(test_fms_vda5050_client test_fms_vda5050_client.cpp)
# target_link_libraries(test_fms_vda5050_client PRIVATE PahoMqttCpp::paho-mqttpp3-shared)
# if(UNIX AND PAHOMQTTC_USE_SSL)
#     find_package(OpenSSL REQUIRED)
#     target_link_libraries(test_fms_vda5050_client PRIVATE OpenSSL::SSL OpenSSL::Crypto)
# endif()

cmake_minimum_required(VERSION 3.10)
project(fms_mqtt_publish)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(fms_mqtt_publish fms_mqtt_publish.cpp)

find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB paho-mqtt3a)

if (NOT PAHO_MQTT_CPP_LIB)
    message(FATAL_ERROR "paho-mqttpp3 library not found")
endif()

if (NOT PAHO_MQTT_C_LIB)
    message(FATAL_ERROR "paho-mqtt3a library not found")
endif()

find_package(Threads REQUIRED)

target_link_libraries(fms_mqtt_publish PRIVATE
    ${PAHO_MQTT_CPP_LIB}
    ${PAHO_MQTT_C_LIB}
    Threads::Threads
)
