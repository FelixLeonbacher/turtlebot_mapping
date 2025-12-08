cmake_minimum_required(VERSION 3.16)
project(turtlebot_mapping LANGUAGES CXX)

# Use C++20
set(CMAKE_CXX_STANDARD 20)

# Use C++17
# set(CMAKE_CXX_STANDARD 17)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/threading

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

# threading/main.cpp
add_executable(threading_main src/threading/main.cpp)
target_link_libraries(threading_main PRIVATE turtlebot_core)
target_include_directories(threading_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)




# get all .cpp files in /testing
file(GLOB TEST_FILES "src/testing/*.cpp")

foreach(TEST_SRC ${TEST_FILES})
    # extract filename without extension
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)

    # create executable for each .cpp file
    add_executable(${TEST_NAME} ${TEST_SRC})

    # link with main library 
    target_link_libraries(${TEST_NAME} PRIVATE turtlebot_core)

    # optional: include src/ for headers
    target_include_directories(${TEST_NAME} PRIVATE src)
endforeach()

