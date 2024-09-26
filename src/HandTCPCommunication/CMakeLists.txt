cmake_minimum_required(VERSION 2.8.3)
project(HandTCPCommunication)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions 
  message_generation
)

add_message_files(
    FILES
    TCPRecvSensorData.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs message_runtime   
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
)
LINK_DIRECTORIES(
  /usr/lib/x86_64-linux-gun 
)

add_library( 
  TCPCommunication_Hand
  src/TCPCommunication_Hand.cpp
)  

file(GLOB handtcpsources 
    "src/*.cpp"
)

add_library(hand_tcp_src ${handtcpsources})   
add_executable(HandCtrl_node src/HandCtrl_node.cpp ${hand_tcp_src})

find_package(jsoncpp)
target_link_libraries(
    HandCtrl_node   
    TCPCommunication_Hand  
    hand_tcp_src 
    ${catkin_LIBRARIES}
    jsoncpp 
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS
  HandCtrl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
