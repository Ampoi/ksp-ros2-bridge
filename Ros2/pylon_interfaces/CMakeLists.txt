cmake_minimum_required(VERSION 3.8)
project(pylon_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StarTrackerState.msg"
  "msg/WheelCommand.msg"
  "msg/WheelState.msg"
  "msg/EngineCommand.msg"
  "msg/EngineState.msg"
  "msg/RcsCommand.msg"
  "msg/RcsState.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/DockingPortCommand.msg"
  "msg/DockingPortState.msg"
  "msg/SeparationCommand.msg"
  "msg/SeparationState.msg"
  "msg/ControlSetpoint.msg"
  "msg/BodyWrenchCommand.msg"
  "msg/ControlAuthorityCommand.msg"
  "msg/ControlAuthorityState.msg"
  "msg/WrenchFeedback.msg"
  "msg/VesselLifecycle.msg"
  "msg/NearbyVessel.msg"
  "msg/NearbyVessels.msg"
  "msg/RelativeTarget.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
