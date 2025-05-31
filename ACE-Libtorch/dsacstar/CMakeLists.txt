cmake_minimum_required(VERSION 3.10)

set(target_name dsacstar)
add_library(
    ${target_name}
    STATIC
    dsacstar_derivative.h
    dsacstar_loss.h
    dsacstar_types.h
    dsacstar_util.h
    dsacstar_util_rgbd.h
    stop_watch.h
    thread_rand.cpp
    thread_rand.h
    dsacstar.cpp
    thread_rand.cpp
    dsacstar.h
)
set_target_properties(${target_name} PROPERTIES CXX_STANDARD 17)


target_link_libraries(
    ${target_name}
    ${TORCH_LIBRARIES} 
    ${OpenCV_LIBS}
)

target_link_options(
        ${target_name}
        PUBLIC
        -fopenmp

)

# set_property(TARGET ${project_name} PROPERTY CXX_STANDARD 17)