# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target livox_interfaces::livox_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${livox_interfaces_TARGETS}.
if(livox_interfaces_TARGETS AND NOT TARGET livox_interfaces::livox_interfaces)
  add_library(livox_interfaces::livox_interfaces INTERFACE IMPORTED)
  set_target_properties(livox_interfaces::livox_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${livox_interfaces_TARGETS}")
endif()
