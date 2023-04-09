cmake_minimum_required(VERSION 3.22)

project(Scheduling VERSION 0.1)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
# set(CMAKE_CXX_COMPILER /usr/bin/clang++)
# set(CMAKE_CXX_CLANG_TIDY /usr/bin/clang-tidy; -header-filter=.*; -checks=*;)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Boost 1.79 REQUIRED COMPONENTS program_options log timer)
file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/greedy/*.cpp)

add_subdirectory(external/json)
add_subdirectory(external/metis)
add_subdirectory(external/toml11)

include_directories(external/json/include)
include_directories(external/metis/include)
include_directories(external/toml11/toml)

add_library(scheduling ${SOURCES})
target_link_libraries(scheduling Boost::log nlohmann_json toml11 metis)

add_executable(opts main.cpp)
target_link_libraries(opts scheduling Boost::program_options Boost::timer)
