cmake_minimum_required(VERSION 3.0.2)
project(topic_tutorial_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    sensor_msgs
    actionlib_msgs
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Sample.msg
  Samples.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Sleep.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

# catkin_package(
# )
# include_directories(
# )
