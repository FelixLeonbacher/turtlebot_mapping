cmake_minimum_required(VERSION 3.16)
project(turtlebot_mapping LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings for GCC/MinGW (w64devkit)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -g)
endif()

# --------------------------------------------
# Collect all source files automatically
# --------------------------------------------

file(GLOB_RECURSE PROJECT_SOURCES
    src/*.cpp
)

# Remove auto-detected EXEs accidentally included
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*\\.exe$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*dummyfile$")

# --------------------------------------------
# Build a STATIC library with all core code
# --------------------------------------------

add_library(turtlebot_core STATIC
    ${PROJECT_SOURCES}
)

# Include directories for headers
target_include_directories(turtlebot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connection
    ${CMAKE_CURRENT_SOURCE_DIR}/src/export
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pathing
)

# Winsock for Windows/w64devkit
if (WIN32)
    target_link_libraries(turtlebot_core PUBLIC ws2_32)
endif()

# --------------------------------------------
# Executables
# --------------------------------------------


# main.cpp
add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE turtlebot_core)

