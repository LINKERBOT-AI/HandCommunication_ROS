cmake_minimum_required(VERSION 2.8.3)
project(CanCtrlHand)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions 
  message_generation
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
  Can_Communication
  src/Can_Communication.cpp
)  

file(GLOB cansources 
    "src/*.cpp"
)

add_library(can_src ${cansources})   
add_executable(Can_Communication_Node src/Can_Communication_Node.cpp ${can_src})
 
target_link_libraries(
    Can_Communication_Node    
    Can_Communication
    can_src 
    ${catkin_LIBRARIES} 
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS
  Can_Communication_Node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
