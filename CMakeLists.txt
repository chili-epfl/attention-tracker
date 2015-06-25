cmake_minimum_required(VERSION 2.8.3)
project(attention_tracker)

add_definitions(-std=c++11)

set(DLIB_PATH "" CACHE PATH "Path to DLIB")
include(${DLIB_PATH}/dlib/cmake)

option(DEBUG "Enable debug visualizations" OFF)
option(WITH_TESTS "Compile sample test application" OFF)
option(WITH_ROS "Build ROS nodes -- Requires OpenCV2!" ON)

if(WITH_ROS)

    find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf
        std_msgs
        visualization_msgs
        sensor_msgs
        cv_bridge
        image_transport
        image_geometry
        )

    include_directories(${catkin_INCLUDE_DIRS})

    catkin_package(
    CATKIN_DEPENDS 
        tf
    DEPENDS OpenCV
    LIBRARIES 
    )
endif()

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(${OpenCV_VERSION} VERSION_GREATER 2.9.0)
    set(OPENCV3 TRUE)
    add_definitions(-DOPENCV3)
endif()



if(DEBUG)
    add_definitions(-DHEAD_POSE_ESTIMATION_DEBUG)
    if(OPENCV3)
        find_package(OpenCV COMPONENTS core imgproc highgui videoio REQUIRED)
    else()
        find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)
    endif()

else()
    find_package(OpenCV COMPONENTS core imgproc REQUIRED)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(head_pose_estimation STATIC src/head_pose_estimation.cpp)
target_link_libraries(head_pose_estimation dlib ${OpenCV_LIBRARIES})

if(WITH_ROS)

    if(OPENCV3)
        message(FATAL_ERROR "OpenCV2 is currently required to build the ROS nodes!")
    endif()

    add_executable(expected_focus src/expected_focus.cpp)
    target_link_libraries(expected_focus ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

    add_executable(estimate src/head_pose_estimation_ros.cpp src/ros_head_pose_estimator.cpp)
    target_link_libraries(estimate head_pose_estimation ${catkin_LIBRARIES})

    install(TARGETS expected_focus estimate
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(FILES
        launch/attention_tracking.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif()

if(WITH_TESTS)
    # visualizations need to be enabled
    add_definitions(-DHEAD_POSE_ESTIMATION_DEBUG)

    add_executable(head_pose_test samples/head_pose_estimation_test.cpp)
    target_link_libraries(head_pose_test head_pose_estimation ${OpenCV_LIBRARIES})

endif()


