cmake_minimum_required(VERSION 2.8.3)
project(attention_tracker)

find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        visualization_msgs
        sensor_msgs)

catkin_package(
  LIBRARIES 
)


add_definitions(-std=c++11)

add_executable(expected_focus src/expected_focus.cpp)
target_link_libraries(expected_focus ${catkin_LIBRARIES})

install(TARGETS expected_focus 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/attention_tracking.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


