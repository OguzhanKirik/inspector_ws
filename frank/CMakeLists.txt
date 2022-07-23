cmake_minimum_required(VERSION 2.8.3)
project(frank)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
)

add_message_files(
  FILES
  baseStruct.msg
  structArray.msg
  profiles_msg.msg
  OperationState.msg
  point.msg
  cluster.msg
  color.msg
)
add_service_files(
  FILES
  ReachCheck.srv
  PathPlan.srv
  StateReq.srv
  CommandPLC.srv
  Connect2DCamera.srv
  Connect3DCamera.srv
  Connect3DCamerax36.srv
  ConnectPLC.srv
  ConnectProfilometer.srv
  Disconnect2DCamera.srv
  Disconnect3DCamera.srv
  Disconnect3DCamerax36.srv
  DisconnectPLC.srv
  DisconnectProfilometer.srv
  Grab2DImage.srv
  Grab3DPointcloud.srv
  Grab3DPointcloudx36.srv
  SendSparseScan.srv
  StopProfi.srv
  StartProfi.srv
  ReadPLC.srv
  UpdateDrillTCP.srv
  UpdateRackLayout.srv
  InitProfi.srv
  SaveCalibScan.srv
  SaveSparseScan.srv
  SparseAlignment.srv
  DenseAlignment.srv
  EnableSaveProfi.srv
  RequestSaveProfi.srv
  RequestStopSaveProfi.srv
  PLCToolInfo.srv
  ExtractionAndClustering.srv
  RapidCommand.srv
  GoToPose.srv
  GetPose.srv
  CallRoutine.srv
  GenerateRangeImage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(abb_libegm REQUIRED)
find_package(abb_librws REQUIRED)
# ### Added later

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIB_DIR}  )
add_definitions(${PCL_DEFINITIONS} )

# ### Added later
##################################
## catkin specific configuration##
##################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime sensor_msgs 
  DEPENDS abb_libegm
  DEPENDS abb_librws 

)

#########
##Build##
#########
include_directories(
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)




#################################
# Build the basic level samples #
#################################


add_executable(${PROJECT_NAME}_pose_pub_2 src/pose_pub_2.cpp)
set_target_properties(${PROJECT_NAME}_pose_pub_2 PROPERTIES OUTPUT_NAME pose_pub_2_velocity_controller_node_node PREFIX "")
add_dependencies(${PROJECT_NAME}_pose_pub_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pose_pub_2 
abb_libegm::abb_libegm 
${catkin_LIBRARIES})



add_executable(${PROJECT_NAME}_rws src/rws_node.cpp)
set_target_properties(${PROJECT_NAME}_rws PROPERTIES OUTPUT_NAME rws_node PREFIX "")
add_dependencies(${PROJECT_NAME}_rws ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rws 
abb_librws::abb_librws 
${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_UpdateDrillservices src/updateDrillServices.cpp)
set_target_properties(${PROJECT_NAME}_UpdateDrillservices PROPERTIES OUTPUT_NAME UpdateDrillservices PREFIX "")
add_dependencies(${PROJECT_NAME}_UpdateDrillservices ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_UpdateDrillservices
${catkin_LIBRARIES})

### Added later
add_executable(${PROJECT_NAME}_rangeImage_services src/rangeImage_services.cpp)
set_target_properties(${PROJECT_NAME}_rangeImage_services PROPERTIES OUTPUT_NAME rangeImage_services PREFIX "")
add_dependencies(${PROJECT_NAME}_rangeImage_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rangeImage_services ${OpenCV_LIBS} ${PCL_LIBRARIES} 
${catkin_LIBRARIES})
### Added later

### Added later
add_executable(${PROJECT_NAME}_rangeImage_client src/rangeImage_client.cpp)
set_target_properties(${PROJECT_NAME}_rangeImage_client PROPERTIES OUTPUT_NAME rangeImage_client PREFIX "")
add_dependencies(${PROJECT_NAME}_rangeImage_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rangeImage_client ${OpenCV_LIBS} ${PCL_LIBRARIES} 
${catkin_LIBRARIES})
### Added later

### Added later
add_executable(${PROJECT_NAME}_rangeimage_node src/rangeimage_node.cpp)
set_target_properties(${PROJECT_NAME}_rangeimage_node PROPERTIES OUTPUT_NAME rangeimage_node PREFIX "")
add_dependencies(${PROJECT_NAME}_rangeimage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rangeimage_node ${OpenCV_LIBS}
${catkin_LIBRARIES})
### Added later


### Added later
add_executable(${PROJECT_NAME}_rangeimage_subscriber src/rangeimage_subscriber.cpp)
set_target_properties(${PROJECT_NAME}_rangeimage_subscriber PROPERTIES OUTPUT_NAME rangeimage_subscriber PREFIX "")
add_dependencies(${PROJECT_NAME}_rangeimage_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rangeimage_subscriber ${OpenCV_LIBS}
${catkin_LIBRARIES})
### Added later

#add_dependencies(plc ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_rws ${catkin_EXPORTED_TARGETS})
# add_dependencies(rws_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
