cmake_minimum_required(VERSION 3.16.3)
set(name ace)

option(BUILD_ANDROID "build Android lib" ON)
option(WITH_OPENCV "link opencv lib" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_C_COMPILER "clang")
# set(CMAKE_CXX_COMPILER "clang++")

project(${name})
set(BUILD_ANDROID ON)

set(WITH_OPENCV ON)
################################################################################
include_directories(/home/sophda/libtorch/pytorch/android-shared/include/torch/csrc/api/include)
include_directories(/home/sophda/libtorch/pytorch/android-shared/include)

set(Torch_DIR /home/sophda/libtorch/pytorch/android-shared/Torch.cmake)
include(${Torch_DIR})
include_directories(${TORCH_INCLUDE_DIRS})
    # if(WITH_OPENCV)
    #     SET(OpenCV_DIR /home/sophda/src/opencv-3.4.16/armv8_build)
    #     find_package(OpenCV 3.4.16 REQUIRED)
    # endif()
# else()
#     set(Torch_DIR /home/sophda/libtorch/x64/libtorch/share/cmake/Torch)
#     find_package(Torch REQUIRED)
#     if(WITH_OPENCV)
#         find_package(OpenCV  REQUIRED)
#     endif()
# endif()
##################################################################################

set(TORCH_INSTALL_PREFIX /home/sophda/libtorch/pytorch/android-shared)
set(TORCH_INCLUDE_DIRS
${TORCH_INSTALL_PREFIX}/include
${TORCH_INSTALL_PREFIX}/include/torch/csrc/api/include)


#########################################################################

##########################################################################


##################################################################################

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

#  add_subdirectory(
#      ${CMAKE_CURRENT_SOURCE_DIR}/dsacstar
#      ${CMAKE_CURRENT_SOURCE_DIR}/dsacstar/build
#  )
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
)


add_library(
    ${name}
    SHARED
    ace.cpp
    ace.hpp
    dsacstar.cpp
    thread_rand.cpp

)

# add_executable(
#     ${name}
#     ace.cpp
#     ace.hpp
#     # interface.cpp
# )


target_link_libraries(
    ${name}
    log
    ${TORCH_LIBRARIES} 
    ${OpenCV_LIBS}
)
target_link_options(
    ${name}
    PUBLIC
    -fopenmp
)
