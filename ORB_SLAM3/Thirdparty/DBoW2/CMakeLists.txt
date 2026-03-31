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

include_directories(    
  /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include
  /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include/opencv
  /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include/opencv2
)

include_directories(
../boost_1_82_0
../boost_1_82_0/boost
/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/DBoW2/DBoW2
)
add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(
  DBoW2 
  ${OpenCV_LIBS}
  boost_LIB
)

