cmake_minimum_required(VERSION 3.8)
project(cogndn_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg" "msg/DetectionArray.msg" "msg/Obstacle.msg" "msg/ObstacleArray.msg"
  "msg/InformationNeed.msg" "msg/CognitiveState.msg" "msg/NamedInterest.msg" "msg/InterestBatch.msg"
  "msg/DataArrival.msg" "msg/EnvelopeEvent.msg" "msg/RouteState.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
