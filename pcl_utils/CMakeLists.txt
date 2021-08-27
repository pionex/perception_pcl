cmake_minimum_required(VERSION 3.5)
project(pcl_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system program_options filesystem python38)
#find_package(VTK REQUIRED)
find_package(lib_perception REQUIRED)

include_directories(include)


set(dependencies
        pcl_conversions
        rclcpp
        sensor_msgs
        geometry_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        EIGEN3
        PCL
        pluginlib
        pcl_msgs
        lib_perception


        )

### Nodelets
#
### Declare the pcl_ros_io library
  add_library(pcl_utils SHARED
  #src/pcl_ros/io/bag_io.cpp
  #src/pcl_ros/io/concatenate_data.cpp
  #src/pcl_ros/io/concatenate_fields.cpp
  #src/pcl_ros/io/io.cpp
  #src/pcl_utils/normviz.cpp
  src/pcl_utils/NormalsVisualizer.cpp
  src/pcl_utils/NormalsGenerator.cpp
)
target_link_libraries(pcl_utils ${PCL_LIBRARIES})
target_compile_definitions(pcl_utils
        PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pcl_utils
        ${dependencies}
        )
rclcpp_components_register_nodes(pcl_utils "pcl_utils::NormalsVisualizer")
rclcpp_components_register_nodes(pcl_utils "pcl_utils::NormalsGenerator")

#set(node_plugins "${node_plugins}pcl_utils::NormViz;$<TARGET_FILE:pcl_utils>\n")


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  #add_rostest_gtest(test_tf_message_filter_pcl tests/test_tf_message_filter_pcl.launch src/test/test_tf_message_filter_pcl.cpp)
  #target_link_libraries(test_tf_message_filter_pcl ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  #add_rostest(samples/pcl_ros/features/sample_normal_3d.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/filters/sample_statistical_outlier_removal.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/filters/sample_voxel_grid.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/segmentation/sample_extract_clusters.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/surface/sample_convex_hull.launch ARGS gui:=false)
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    pcl_utils
  EXPORT export_pcl_utils
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

#install(DIRECTORY plugins samples
#  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(pcl_utils)
ament_export_dependencies(${dependencies})
ament_export_targets(export_pcl_utils HAS_LIBRARY_TARGET)
ament_package()
