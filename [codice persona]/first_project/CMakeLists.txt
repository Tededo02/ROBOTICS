find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  sector_times.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)