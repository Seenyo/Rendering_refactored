cmake_minimum_required(VERSION 3.22)
project(Rendering2022-refactoring)

set(CMAKE_CXX_STANDARD 17)
include_directories("C:/Windows/Eigen/eigen-3.4.0")

add_executable(Rendering2022-refactoring main.cpp Camera.cpp Camera.h RayHitInfo.h TriangleObj.h Film.cpp Film.h utils.h utils.cpp TriangleObj.cpp Ray.h constants.h)
target_link_libraries(Rendering2022-refactoring libopengl32.a libglu32.a libfreeglut.a)
