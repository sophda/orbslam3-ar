# set(CMAKE_TOOLCHAIN_FILE "/home/sophda/src/android-ndk-r25c/build/cmake/android.toolchain.cmake" )

cmake_minimum_required(VERSION 3.4.1)
SET(PROJECT_NAME slamAR)

project(${PROJECT_NAME})


set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "19" CACHE STRING "")

# set(CMAKE_CXX_COMPILER clang++)
# set(CMAKE_C_COMPILER clang)
# set(ANDROID_NDK "/home/sophda/src/android-ndk-r25c" CACHE STRING "")
# add_compile_definitions(ANDROID_ABI arm64-v8a)
SET(OpenCV_DIR /home/sophda/src/opencv-3.4.16/armv8_build)
find_package(OpenCV 3.4.16 REQUIRED)

find_library(log-lib log android)

set(boost_include_dir /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/boost_1_82_0)
add_library(boost_LIB INTERFACE)
target_include_directories(
    boost_LIB
    INTERFACE
    ${boost_include_dir}
)
link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/boost_1_82_0/arm64_lib)

include_directories(    
    /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include
    /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include/opencv
    /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include/opencv2
)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/eigen-3.4.0
)


include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus/sophus)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/include)
include_directories(
    /home/sophda/project/OrbSlam3AR/ORB_SLAM3
    /home/sophda/project/OrbSlam3AR/ORB_SLAM3/include/CameraModels
    /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus
    /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/boost_1_82_0
)
add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3
        ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/build
)

# this target
option(BUILD_ANDROID "build andorid lib" TRUE)
if(BUILD_ANDROID)
    add_compile_definitions(ANDROID)
endif()


add_library(${PROJECT_NAME} SHARED native.cpp  PoseMerge.hpp PoseMerge.cpp ndkcamera.cpp ndkcamera.hpp )
target_link_libraries(
    ${PROJECT_NAME} 
    ORB_SLAM3
    ${OpenCV_LIBS}
    android
    log
    libboost_atomic-clang-mt-a64-1_82.a
    libboost_serialization-clang-mt-a64-1_82.a
    libboost_system-clang-mt-a64-1_82.a
    mediandk
    camera2ndk
)