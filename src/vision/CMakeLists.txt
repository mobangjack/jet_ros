cmake_minimum_required(VERSION 2.8.3)
project(vision)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    tf
    cv_bridge
    image_transport
    roscpp
    message_generation
    std_msgs 
    geometry_msgs
    cmake_modules
)

find_package(OpenCV REQUIRED)
#find_package(OpenMP REQUIRED)
#find_package(Eigen REQUIRED)

#FIND_PACKAGE(OpenMP REQUIRED)
#if(OPENMP_FOUND)
#message("OPENMP FOUND")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

# add_message_files(
#   FILES
# )

# add_service_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs  # Or other packages containing msgs
# )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES apriltags
#    DEPENDS Eigen
)

include_directories(
    include
    aruco/include
    ar_sys/include
    circle/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_library(aruco
	aruco/src/arucofidmarkers.cpp
	aruco/src/cvdrawingutils.cpp
	aruco/src/cameraparameters.cpp
	aruco/src/board.cpp
	aruco/src/marker.cpp
	aruco/src/boarddetector.cpp
	aruco/src/markerdetector.cpp
	aruco/src/ar_omp.cpp
	aruco/src/subpixelcorner.cpp
)

target_link_libraries(aruco
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(ar_sys_utils
    ar_sys/src/utils.cpp
)

target_link_libraries(ar_sys_utils
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  aruco
)

add_library(circle_detector circle/src/circle_detector.cpp)

target_link_libraries(circle_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(circle_detector_test
    circle/src/circle_detector_test.cpp
)

target_link_libraries(circle_detector_test
    ${OpenCV_LIBRARIES} 
    circle_detector
)

add_executable(vision
    src/vision.cpp
)

# add_dependencies(vision vision_generate_messages_cpp)

target_link_libraries(vision 
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBRARIES} 
    aruco
    ar_sys_utils
    circle_detector
)
