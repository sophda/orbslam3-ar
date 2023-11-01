cmake_minimum_required(VERSION 3.4.1)
SET(PROJECT_NAME slamAR)
project(${PROJECT_NAME})
# set(CMAKE_CXX_FLAGS "-O3 -Wall -W -fPIC -fpermissive")
# SET(CMAKE_C_FLAGS "-O3 -Wall -W -fPIC")

set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "19" CACHE STRING "")
# set(ANDROID_NDK "/home/sophda/src/android-ndk-r25c" CACHE STRING "")
# set(CMAKE_TOOLCHAIN_FILE "/home/sophda/src/android-ndk-r25c/build/cmake/android.toolchain.cmake" CACHE STRING "")
# add_compile_definitions(ANDROID_ABI arm64-v8a)

# set(OpenCV_DIR /home/sophda/src/opencv-3.4.16/build/install/sdk/native/jni)
# find_package(OpenCV 3.0 QUIET)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/include)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/include/CameraModels)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/DBoW2)
# 
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/DBoW2/DBoW2)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/eigen-3.4.0)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/eigen-3.4.0/Eigen)

include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/g2o)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/g2o/g2o)

include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus/sophus)
include_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus)
include_directories(
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/openssl
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/openssl/include
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/ndk_25_boost_1.79.0/include
/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Boost-for-Android-master/boost_1_82_0
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Boost-for-Android-master/boost_1_82_0/boost
/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/Sophus
# opencv
/home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include

)

link_directories(/home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/libs/arm64-v8a)
link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/g2o/arm64_build)
link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/DBoW2/armv8a_build)
link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build)



add_library(${PROJECT_NAME} SHARED fun.cpp   )
target_link_libraries(${PROJECT_NAME} 


libg2o.so
libDBoW2.so
libORB_SLAM3.so

libopencv_calib3d.so  libopencv_features2d.so  libopencv_imgcodecs.so  libopencv_objdetect.so  libopencv_stitching.so  libopencv_videoio.so
libopencv_core.so     libopencv_flann.so       libopencv_imgproc.so    libopencv_photo.so      libopencv_superres.so   libopencv_videostab.so
libopencv_dnn.so      libopencv_highgui.so     libopencv_ml.so         libopencv_shape.so      libopencv_video.so
)