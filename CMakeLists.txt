cmake_minimum_required(VERSION 2.14)
project(rtc_in_one_weekend)
add_compile_options(-std=c++17)
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
# add_library(rtc src/rigid_body.cpp src/material.cpp)
add_library(rtc src/material.cpp src/rigid_body.cpp  src/tinygltf_impl.cpp)
target_link_libraries(rtc PUBLIC tinygltf mxm)
target_include_directories(rtc PUBLIC third_party/mxm/inc)

add_subdirectory(app)

if(BUILD_TEST)
    add_subdirectory(tests)
endif(BUILD_TEST)