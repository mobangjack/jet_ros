cmake_minimum_required(VERSION 2.8.3)

set (PROJ_NAME circle_detector)

project(${PROJ_NAME})

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_library(circle_detector src/circle_detector.cpp)

add_executable(${PROJ_NAME}_test
    src/circle_detector_test.cpp
)

target_link_libraries(${PROJ_NAME}_test ${OpenCV_LIBRARIES} circle_detector)

