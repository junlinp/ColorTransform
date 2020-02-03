cmake_minimum_required(VERSION 3.0)
project(ColorTransform)
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS ${OpenCV_INCLUDE_DIRS})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include/eigen3
    )

add_subdirectory(src)
add_executable(generate_dataset generate_dataset.cc)
target_link_libraries(generate_dataset ${OpenCV_LIBS})

add_executable(color_transform_test color_transform_test.cc file_interface.cc color_transform.cc)
target_link_libraries(color_transform_test ${OpenCV_LIBS} quadprog)