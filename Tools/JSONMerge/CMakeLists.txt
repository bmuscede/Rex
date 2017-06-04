cmake_minimum_required(VERSION 3.7)
project(JSONMerge)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost 1.58 COMPONENTS system program_options filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

set(SOURCE_FILES main.cpp)
add_executable(JSONMerge ${SOURCE_FILES})

target_link_libraries(JSONMerge ${Boost_LIBRARIES})