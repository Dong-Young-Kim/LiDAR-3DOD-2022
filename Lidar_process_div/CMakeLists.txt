cmake_minimum_required(VERSION 3.0.2)
project(Lidar_process_div)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(filter_div src/Lidar_code/Lidar_filter.cpp)
target_link_libraries(filter_div ${catkin_LIBRARIES})
#add_dependencies(filter_div ${catkin_EXPORTED_TARGETS})

add_library(etc_div src/Lidar_code/Lidar_etc.cpp)
target_link_libraries(etc_div ${catkin_LIBRARIES})
#add_dependencies(etc_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(msg_div src/Lidar_code/Lidar_msg.cpp)
target_link_libraries(msg_div ${catkin_LIBRARIES})
#add_dependencies(msg_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(Lidar_code src/Lidar_code/Lidar_code.cpp)
target_link_libraries(Lidar_code ${catkin_LIBRARIES} pcl_visualization filter_div etc_div msg_div)
#add_dependencies(Lidar_code ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_library(decalre_div src/extra_code/Lidar_declare.h)
#target_link_libraries(decalre_div ${catkin_LIBRARIES})

#add_library(param_div src/extra_code/set_param.h)
#target_link_libraries(param_div ${catkin_LIBRARIES})



add_executable(Lidar_ROI src/node_code/Lidar_input_ROI.cpp)
target_link_libraries(Lidar_ROI Lidar_code)

add_executable(Lidar_ransac src/node_code/Lidar_ransac.cpp)
target_link_libraries(Lidar_ransac Lidar_code)

add_executable(Lidar_Clustering src/node_code/Lidar_Clustering.cpp)
target_link_libraries(Lidar_Clustering Lidar_code)

add_executable(Lidar_visual src/node_code/Lidar_visual.cpp)
target_link_libraries(Lidar_visual Lidar_code)



