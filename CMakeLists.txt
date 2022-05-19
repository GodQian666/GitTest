cmake_minimum_required(VERSION 3.22)
project(GitTest00)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
add_executable(GitTest00
                main.cpp
                src/swap.cpp)
