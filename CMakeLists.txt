cmake_minimum_required(VERSION 2.14)
project(rtc_in_one_weekend)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_SYSTEM_NAME MATCHES "^Android")
    set(THREAD_LIB stdc++)
elseif(CMAKE_SYSTEM_NAME MATCHES "^Linux")
    set(THREAD_LIB pthread)
endif()

# include_directories(inc)
# add_library(rtc src/io.cpp src/material.cpp src/rigid_body.cpp src/rotation.cpp)
add_subdirectory(third_party)

include_directories(inc)
add_library(rtc src/material.cpp src/rigid_body.cpp src/rotation.cpp src/tinygltf_impl.cpp)
target_link_libraries(rtc PUBLIC tinygltf)

add_subdirectory(app)

if(BUILD_TEST)
    add_subdirectory(tests)
endif(BUILD_TEST)