add_executable(exercise_5 exercise_5.cpp)
target_link_libraries(exercise_5 rtc ${THREAD_LIB})

add_executable(exercise_5_2d exercise_5_2d.cpp)
target_link_libraries(exercise_5_2d rtc ${THREAD_LIB})

add_executable(exercise_5_4d exercise_5_4d.cpp)
target_link_libraries(exercise_5_4d rtc ${THREAD_LIB})

add_executable(one_ray_trace one_ray_trace.cpp)
target_link_libraries(one_ray_trace rtc )