cmake_minimum_required(VERSION 3.20)
project(raytracing-course-hw)

set(CMAKE_CXX_STANDARD 17)

set(EXE raytracer)

add_executable(${EXE} src/main.cpp)
target_include_directories(${EXE} PRIVATE "include")
