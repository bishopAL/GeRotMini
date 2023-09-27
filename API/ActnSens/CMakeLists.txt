cmake_minimum_required(VERSION 2.8.9)

PROJECT(sensors)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include/dynamixel)
include_directories(${PROJECT_SOURCE_DIR}/include/INA219)
include_directories(${PROJECT_SOURCE_DIR}/include/SVnPUMPnIMU)
include_directories(/usr/local/include/)
include_directories(/usr/include/ )
link_directories("/usr/local/lib")
link_directories("/usr/lib")

message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
if(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
set( DYNAMIXELLIB "dxl_mac_cpp")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set( DYNAMIXELLIB "dxl_x64_cpp")
else()
set( DYNAMIXELLIB "dxl_sbc_cpp")
endif()

file(GLOB DYNAMIXEL_SRC
     "./src/dynamixel/*.cpp"
     "./src/dynamixel/*.c"
)

file(GLOB INA219_SRC
     "./src/INA219/*.cpp"
     "./src/INA219/*.c"
)

file(GLOB SVPUMPIMU_SRC
     "./src/SVnPUMPnIMU/*.cpp"
     "./src/SVnPUMPnIMU/*.c"
)

file(GLOB MAIN
     "./src/*.cpp"
     "./src/*.c"
)

ADD_EXECUTABLE(test ${MAIN} ${SVPUMPIMU_SRC} ${DYNAMIXEL_SRC} ${INA219_SRC} ) 
target_link_libraries(test -lwiringPi)
target_link_libraries(test ${DYNAMIXELLIB} )
