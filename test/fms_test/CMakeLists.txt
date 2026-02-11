cmake_minimum_required(VERSION 3.10)
project(TestFmsVda5050Client CXX)

# Paho MQTT C++ 라이브러리 찾기
find_package(PahoMqttCpp REQUIRED)

# nlohmann/json 추가 (단일 헤더 파일 사용 시)
# option(USE_NLOHMANN_JSON_SINGLE_HEADER "Use nlohmann/json single header file" ON)
# if(USE_NLOHMANN_JSON_SINGLE_HEADER)
#     # nlohmann/json.hpp 파일 경로를 include_directories에 추가
#     include_directories(/path/to/nlohmann/json/single_include) # 실제 경로로 변경
# endif()

# 또는 nlohmann_json 설치 후 find_package (vcpkg 등으로 설치 시)
# find_package(nlohmann_json CONFIG REQUIRED) 

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/json)

add_executable(test_fms_vda5050_client test_fms_vda5050_client.cpp)

# 타겟에 Paho MQTT C++ 라이브러리 링크
# target_link_libraries(test_fms_vda5050_client PRIVATE PahoMqttCpp::mqttcpp)
target_link_libraries(test_fms_vda5050_client PRIVATE PahoMqttCpp::paho-mqttpp3-shared)
# nlohmann_json 링크
# target_link_libraries(test_fms_vda5050_client PRIVATE nlohmann_json::nlohmann_json)

# (리눅스에서만) Paho C++가 SSL을 사용하는 경우, OpenSSL 링크
if(UNIX AND PAHOMQTTC_USE_SSL)
    find_package(OpenSSL REQUIRED)
    target_link_libraries(test_fms_vda5050_client PRIVATE OpenSSL::SSL OpenSSL::Crypto)
endif()