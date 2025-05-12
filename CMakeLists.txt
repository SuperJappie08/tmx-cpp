# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

# Project name
project(tmx_cpp)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS system thread)

add_subdirectory(libs/async_serial)

# Add executable
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libs/async_serial/ SUB_SOURCES)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/async_serial/include/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(tmx_cpp
  STATIC
  src/tmx.cpp
  src/tmx_util.cpp
  src/serialization.cpp
  src/modules.cpp
  src/modules/HiwonderServo.cpp
  src/modules/PCA9685.cpp
  src/modules/SSD1306_oled.cpp
  src/sensors.cpp
  src/sensors/ADXL345.cpp
  src/sensors/INA226.cpp
  src/sensors/MPU9250.cpp
  src/sensors/VEML6040.cpp
)

# Link Boost libraries
# TODO: is Boost::thread necessary?
target_link_libraries(tmx_cpp Boost::system Boost::thread async_serial)

install(TARGETS tmx_cpp
  # EXPORT tmx_cpp-export
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/
  TYPE INCLUDE
  FILES_MATCHING PATTERN "*.hpp")

add_executable(${PROJECT_NAME}_test_serialization
  test/test_serialization.cpp
)

set_property(TARGET tmx_cpp ${PROJECT_NAME}_test_serialization PROPERTY CXX_STANDARD 20)
# set_target_properties(tmx_cpp ${PROJECT_NAME}_test_serialization
#                       PROPERTIES CXX_STANDARD 20
#                       CMAKE_CXX_FLAGS )
set_property(TARGET tmx_cpp ${PROJECT_NAME}_test_serialization APPEND PROPERTY CMAKE_CXX_FLAGS "-Wswitch-enum") # -Wall -Wextra
# -Wpedantic -Werror")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_definitions(_LIBCPP_ENABLE_CXX20_REMOVED_TYPE_TRAITS _LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)
endif()


target_link_libraries(${PROJECT_NAME}_test_serialization tmx_cpp)

add_test(NAME test_serialization
    COMMAND ${PROJECT_NAME}_test_serialization)
