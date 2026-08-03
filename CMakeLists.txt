cmake_minimum_required(VERSION 3.16)
project(direct_sonar_odometry)

# C++17, as in the original catkin build.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Ubuntu 26.04 ships VTK 9.5, whose exported CMake targets reference
# JsonCpp::JsonCpp, but nothing in the PCL -> VTK config chain ever calls
# find_package(JsonCpp).  pcl_conversions pulls in the full PCL (and therefore
# VTK), so pull in jsoncpp's own config first - it defines the namespaced alias
# and keeps CMake's generate step from failing on an unresolved target.
find_package(jsoncpp QUIET)
find_package(pcl_conversions REQUIRED)
# Only pcl_common is needed (PointCloud, PointXYZ, transformPointCloud); linking
# the full ${PCL_LIBRARIES} would drag in pcl_visualization and all of VTK.
find_package(PCL REQUIRED COMPONENTS common)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)

set(DISO_G2O_LIBS
  g2o::core
  g2o::stuff
  g2o::types_sba
  g2o::types_slam3d
  g2o::solver_dense
  g2o::solver_eigen
)

set(DISO_ROS_LIBS
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport
  message_filters::message_filters
  pcl_conversions::pcl_conversions
  tf2::tf2
  tf2_ros::tf2_ros
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(direct_sonar_odometry_lib SHARED
  src/System.cpp
  src/Frame.cpp
  src/MapPoint.cpp
  src/Track.cpp
  src/OptimizationType.cpp
  src/LocalMapping.cpp
)
target_include_directories(direct_sonar_odometry_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(direct_sonar_odometry_lib PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_definitions(direct_sonar_odometry_lib PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(direct_sonar_odometry_lib PUBLIC
  ${DISO_ROS_LIBS}
  ${DISO_G2O_LIBS}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES}
  Eigen3::Eigen
  Sophus::Sophus
)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(aracati2017_node src/run_aracati2017.cpp)
target_link_libraries(aracati2017_node PRIVATE direct_sonar_odometry_lib)

add_executable(direct_sonar_odometry_node src/main.cpp)
target_link_libraries(direct_sonar_odometry_node PRIVATE direct_sonar_odometry_lib)

# Dead-reckoning odometry prior for the Aracati2017 bag; see src/CmdVelOdom.cpp.
add_executable(cmd_vel_odom src/CmdVelOdom.cpp)
target_link_libraries(cmd_vel_odom PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

add_executable(repub_gt src/RepubGT.cpp)
target_link_libraries(repub_gt PRIVATE
  ${DISO_ROS_LIBS}
  Eigen3::Eigen
)

add_executable(traj_align src/traj_se3_align.cpp)
target_include_directories(traj_align PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(traj_align PRIVATE
  ${DISO_ROS_LIBS}
  ${DISO_G2O_LIBS}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Sophus::Sophus
)

# Standalone prototypes kept from the original repository.  They are not part of
# the pipeline, but they are built so that they cannot silently rot.
add_executable(direct_semidense_node src/direct_semidense_ros.cpp)
target_link_libraries(direct_semidense_node PRIVATE direct_sonar_odometry_lib)

add_executable(direct_semidense_test src/direct_semidense.cpp)
target_include_directories(direct_semidense_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(direct_semidense_test PRIVATE
  ${DISO_G2O_LIBS} ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(pyramid_test src/pyramid_test.cpp)
target_include_directories(pyramid_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pyramid_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS direct_sonar_odometry_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  aracati2017_node
  direct_sonar_odometry_node
  cmd_vel_odom
  repub_gt
  traj_align
  direct_semidense_node
  direct_semidense_test
  pyramid_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The upstream sources predate the ament style guides; only run the license
  # and xmllint checks.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs visualization_msgs
  cv_bridge image_transport message_filters tf2 tf2_ros pcl_conversions
)

ament_package()
