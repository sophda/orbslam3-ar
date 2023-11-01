cmake_minimum_required(VERSION 3.4.1)
SET(PROJECT_NAME test)
project(test)

add_library(${PROJECT_NAME} SHARED fun.cpp)
