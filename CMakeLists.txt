cmake_minimum_required(VERSION 3.20)
project(raytracing-course-hw)


set(CMAKE_CXX_STANDARD 20)

set(EXE raytracer)

add_executable(${EXE} src/main.cpp src/scene.h src/geometry.h src/image.h src/raytracer.h src/bvh.h src/json.hpp src/generated/vectors.generated.inline.h)
target_include_directories(${EXE} PRIVATE include)
