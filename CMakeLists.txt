cmake_minimum_required(VERSION 2.14)
project(rtc_in_one_weekend)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)

include_directories(inc)
add_library(rtc src/io.cpp src/material.cpp src/rigid_body.cpp src/rotation.cpp)

add_executable(write_ppm app/write_ppm.cpp)
target_link_libraries(write_ppm rtc)

add_executable(play_vec3 app/play_vec3.cpp)
target_link_libraries(play_vec3 rtc)

add_executable(exercise_4 app/exercise_4.cpp)
target_link_libraries(exercise_4 rtc)

add_executable(exercise_5 app/exercise_5.cpp)
target_link_libraries(exercise_5 rtc pthread)
