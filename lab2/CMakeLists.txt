cmake_minimum_required(VERSION 3.10)
project(Rasterizer)
set(CMAKE_CXX_STANDARD 17)
add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
