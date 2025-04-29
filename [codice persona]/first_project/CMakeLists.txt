cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  message_generation
)

# Aggiungi i file dei messaggi
add_message_files(
  FILES
  sector_times.msg
)

# Genera i messaggi (inclusi quelli da std_msgs e geometry_msgs)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf tf2 tf2_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Aggiungi gli eseguibili
add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

add_executable(sector_times src/sector_times.cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})



## Installazione degli eseguibili
install(TARGETS odometer gps_odometer sector_times
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
