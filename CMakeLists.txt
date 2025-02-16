cmake_minimum_required(VERSION 3.20)
project(raytracing-course-hw)


set(CMAKE_CXX_STANDARD 17)

set(EXE raytracer)

add_executable(${EXE} src/main.cpp src/scene.h src/geometry.h src/image.h src/raytracer.h src/generated/vectors.generated.inline.h)
target_include_directories(${EXE} PRIVATE include)
