cmake_minimum_required(VERSION 2.8.9)

PROJECT(sensors)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include )
include_directories(/usr/local/include/)
include_directories(/usr/include/ )
link_directories("/usr/local/lib")
link_directories("/usr/lib")

message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

ADD_EXECUTABLE(testSV ./src/setSV_example.cpp ./src/api.cpp ./src/i2c.c)
ADD_EXECUTABLE(testIMU ./src/readIMU_example.c ./src/wit_c_sdk.c ./src/i2c.c)

target_link_libraries(testSV -lwiringPi)
target_link_libraries(testIMU)