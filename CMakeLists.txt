cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 11)

project(HYBRID_A_STAR)
include_directories(include)
set(sources src/main.cpp src/hybrid-a-star.cpp)
add_executable(HYBRID_A_STAR ${sources})


