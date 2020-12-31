cmake_minimum_required(VERSION 3.2)

add_executable(test_main test_main.cpp)
target_include_directories(test_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../inc)
target_link_libraries(test_main rtc)
