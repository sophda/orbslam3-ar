cmake_minimum_required(VERSION 3.4.1)
SET(PROJECT_NAME test)
project(test)
set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "19" CACHE STRING "")

include_directories(/home/sophda/project/OrbSlam3AR/test_static)

add_library(${PROJECT_NAME} STATIC fun.cpp)
