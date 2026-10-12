cmake_minimum_required(VERSION 3.20)
project(prosthetic_joint_pid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(prosthetic_joint_pid src/main.cpp)

if(MSVC)
  target_compile_options(prosthetic_joint_pid PRIVATE /W4 /permissive-)
else()
  target_compile_options(prosthetic_joint_pid PRIVATE -Wall -Wextra -Wpedantic)
endif()
