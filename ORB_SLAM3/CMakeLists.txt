cmake_minimum_required(VERSION 3.4.1)
project(ORB_SLAM3)

set(ANDROID_ABI "arm64-v8a" CACHE STRING "")
set(ANDROID_PLATFORM "android-19" CACHE STRING "")

SET(CMAKE_BUILD_TYPE Release)



MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
include(CheckCXXCompilerFlag)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories("/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/eigen-3.4.0")
include_directories("/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/eigen-3.4.0/Eigen")
include_directories("/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/eigen-3.4.0/Eigen/src")
include_directories("/home/sophda/project/OrbSlam3AR/ORB_SLAM3")
include_directories(
/home/sophda/project/OrbSlam3AR/ORB_SLAM3
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
/home/sophda/project/OrbSlam3AR/ORB_SLAM3/include
/home/sophda/project/OrbSlam3AR/ORB_SLAM3/include/CameraModels
)

add_subdirectory(
   ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2
   ${CMAKE_CURRENT_SOURCE_DIR}//Thirdparty/DBoW2/build
)

add_subdirectory(
   ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o
   ${CMAKE_CURRENT_SOURCE_DIR}//Thirdparty/g2o/build
)


link_directories(/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/boost_1_82_0/arm64_lib)
add_library(${PROJECT_NAME} STATIC
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
# src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
# src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
# src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc

)

# set_target_properties(ORB_SLAM3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES 
# "/home/sophda/project/OrbSlam3AR/ORB_SLAM3/include ;
# /home/sophda/project/OrbSlam3AR/ORB_SLAM3/include/CameraModels")
# opencv
# /home/sophda/src/opencv-3.4.16/armv8_build/install/sdk/native/jni/include


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
DBoW2
g2o
boost_LIB
libboost_atomic-clang-mt-a64-1_82.a
libboost_serialization-clang-mt-a64-1_82.a
libboost_system-clang-mt-a64-1_82.a
)

