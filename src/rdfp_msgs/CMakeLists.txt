cmake_minimum_required(VERSION 3.8)
project(rdfp_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartSession.srv"
  "srv/StopSession.srv"
  "srv/SetString.srv"
  "srv/GetSessionState.srv"
  "srv/StopEpisode.srv"
  "srv/ResetScene.srv"
  "msg/SessionCommand.msg"
  "msg/GripperCommand.msg"
  "msg/TargetJointStates.msg"
  "msg/ClutchState.msg"
  "msg/GripperState.msg"
  "msg/SceneObject.msg"
  "msg/SceneObjects.msg"
  "msg/MotionTrajectory.msg"
  "msg/MotionEvent.msg"
  DEPENDENCIES std_msgs trajectory_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime std_msgs trajectory_msgs geometry_msgs builtin_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
