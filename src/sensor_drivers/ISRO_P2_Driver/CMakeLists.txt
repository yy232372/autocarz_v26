cmake_minimum_required(VERSION 3.8)
project(ISRO_P2_Driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ===========================================
# C Driver Library
# ===========================================
add_library(ISRO_P2_Driver_lib SHARED
  src/ISRO_P2_Driver.c
)
target_include_directories(ISRO_P2_Driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ISRO_P2_Driver_lib pthread m)

# ===========================================
# ROS2 Node Executable
# ===========================================
add_executable(ISRO_P2_Driver_node 
  src/ISRO_P2_Driver_node.cpp
)
target_include_directories(ISRO_P2_Driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ISRO_P2_Driver_node 
  ISRO_P2_Driver_lib
)
ament_target_dependencies(ISRO_P2_Driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
)

# ===========================================
# Install
# ===========================================

# [핵심 수정] 라이브러리는 lib/에 설치 (ROS2가 찾을 수 있는 표준 경로)
install(TARGETS ISRO_P2_Driver_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 실행파일은 lib/${PROJECT_NAME}/에 설치 (ros2 run으로 실행)
install(TARGETS ISRO_P2_Driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Python 스크립트 설치
install(PROGRAMS
  scripts/ntrip.py
  scripts/ISRO_P2_Config.py
  scripts/ISRO_P2_GUI.py
  DESTINATION lib/${PROJECT_NAME}
)

# 바이너리 데이터(ssl_data.bin) 설치 (share 폴더 권장)
install(FILES
  scripts/ssl_data.bin
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY
  include/
  DESTINATION include
)

# Launch, Config 파일 설치
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ===========================================
# Export (다른 패키지에서 이 라이브러리 사용 가능하도록)
# ===========================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(ISRO_P2_Driver_lib)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
