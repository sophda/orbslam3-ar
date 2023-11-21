cmake_minimum_required(VERSION 3.4.1)
SET(PROJECT_NAME ds)
project(${PROJECT_NAME})
set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "19" CACHE STRING "")

include_directories(/home/sophda/project/OrbSlam3AR/test_static)
link_directories(/home/sophda/project/OrbSlam3AR/test_static/build)
add_library(${PROJECT_NAME} SHARED call.cpp)

target_link_libraries(${PROJECT_NAME} 
libtest.a)
