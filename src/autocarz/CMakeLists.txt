cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(autocarz LANGUAGES C CXX)

# ==============================================================================
# 1. C++ 컴파일 설정
# ==============================================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ==============================================================================
# 2. ROS 2 및 외부 라이브러리 탐색
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(
  PCL REQUIRED
  COMPONENTS
    common
    io
    filters
    registration
    segmentation
    sample_consensus
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ==============================================================================
# 3. 사용자 정의 메시지 생성
# ==============================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CtrlCmd.msg"
  "msg/Erp42Odom.msg"
  "msg/Erp42State.msg"
  "msg/WaypointInfo.msg"

  DEPENDENCIES
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# ==============================================================================
# 4. DBSCAN 라이브러리
# ==============================================================================

add_library(dbscan STATIC
  src/dbscan.cpp
)

target_include_directories(dbscan
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ==============================================================================
# 5. C++ ROS 2 실행 파일 공통 설정 함수
# ==============================================================================

function(add_autocarz_node TARGET_NAME SOURCE_FILE)

  add_executable(
    ${TARGET_NAME}
    ${SOURCE_FILE}
  )

  target_include_directories(
    ${TARGET_NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${PCL_INCLUDE_DIRS}
  )

  ament_target_dependencies(
    ${TARGET_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    ament_index_cpp
    pcl_conversions
  )

  target_link_libraries(
    ${TARGET_NAME}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    "${cpp_typesupport_target}"
  )

endfunction()

# ==============================================================================
# 6. C++ ROS 2 노드 등록
# ==============================================================================

add_autocarz_node(
  roi_in
  src/roi_in.cpp
)

add_autocarz_node(
  roi_out
  src/roi_out.cpp
)

add_autocarz_node(
  cluster_in
  src/cluster_in.cpp
)

add_autocarz_node(
  cluster_out
  src/cluster_out.cpp
)

add_autocarz_node(
  ransac_in
  src/ransac_in.cpp
)

add_autocarz_node(
  ransac_out
  src/ransac_out.cpp
)

# DBSCAN을 사용하는 실행 파일과 라이브러리 연결
target_link_libraries(
  cluster_in
  dbscan
)

target_link_libraries(
  cluster_out
  dbscan
)

# ==============================================================================
# 7. C++ 실행 파일 설치
# ==============================================================================

install(
  TARGETS
    roi_in
    roi_out
    cluster_in
    cluster_out
    ransac_in
    ransac_out

  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    dbscan

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ==============================================================================
# 8. Python 실행 파일 설치
# ==============================================================================

install(
  PROGRAMS
    scripts/ERP42RQT.py
    scripts/decision_making.py
    scripts/main.py
    scripts/main_isro.py
    scripts/online.py
    scripts/online_isro.py
    scripts/sensor_data.py
    scripts/sensor_data_isro.py
    scripts/serialControl.py
    scripts/utilities.py
    scripts/vehicle_control.py

  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# 9. Launch, RViz, 경로 데이터 및 메시지 원본 설치
# ==============================================================================

install(
  DIRECTORY
    launch
    rviz
    path
    msg
    scripts

  DESTINATION share/${PROJECT_NAME}
  PATTERN ".~lock*" EXCLUDE
)

# ==============================================================================
# 10. 의존성 내보내기
# ==============================================================================

ament_export_include_directories(include)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  ament_index_cpp
  pcl_conversions
)



ament_package()
