cmake_minimum_required(VERSION 3.5)
project(turtlebot_flatland)

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(ament_cmake REQUIRED)


###########
## Build ##
###########

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY robot
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
