cmake_minimum_required(VERSION 3.4.1)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "android-19" CACHE STRING "")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


# find_package(OpenCV 4 QUIET)
# if(NOT OpenCV_FOUND)
#    find_package(OpenCV 3.0 QUIET)
#    if(NOT OpenCV_FOUND)
#       message(FATAL_ERROR "OpenCV > 3.0 not found.")
#    endif()
# endif()

# set(OpenCV_DIR /home/sophda/src/opencv-3.4.16/build/install/sdk/native/jni)
# find_package(OpenCV 3.0 QUIET)

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/ndk_25_boost_1.79.0/include/
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/ndk_25_boost_1.79.0/include/boost
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/ndk_25_boost_1.79.0/include/boost/serialization
../boost_1_82_0
../boost_1_82_0/boost
# opencv
/home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include
)
link_directories(/home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/libs/arm64-v8a)
# link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/ndk_25_boost_1.79.0/libs/armeabi-v7a/)
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 
libopencv_calib3d.so  libopencv_features2d.so  libopencv_imgcodecs.so  libopencv_objdetect.so  libopencv_stitching.so  libopencv_videoio.so
libopencv_core.so     libopencv_flann.so       libopencv_imgproc.so    libopencv_photo.so      libopencv_superres.so   libopencv_videostab.so
libopencv_dnn.so      libopencv_highgui.so     libopencv_ml.so         libopencv_shape.so      libopencv_video.so
)

