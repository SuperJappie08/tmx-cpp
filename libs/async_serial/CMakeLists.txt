cmake_minimum_required(VERSION 3.1)
project(async_serial)

## Target
set(CMAKE_CXX_STANDARD 11)

include_directories(include/)

add_library(${PROJECT_NAME} STATIC src/AsyncSerial.cpp src/BufferedAsyncSerial.cpp)

## Link libraries
set(BOOST_LIBS date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
