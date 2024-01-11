# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum") # -Wall -Wextra -Wpedantic -Werror")
# Project name
project(tmx_cpp)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS system thread)

# Add executable
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libs/async_serial/ SUB_SOURCES)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/async_serial/ ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_executable(tmx_cpp src/main.cpp src/tmx.cpp src/sensors.cpp ${SUB_SOURCES})

# Link Boost libraries
target_link_libraries(tmx_cpp Boost::system Boost::thread)
install(TARGETS tmx_cpp
  DESTINATION lib/${PROJECT_NAME})
