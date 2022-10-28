######################################
cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

project(launch_program)


find_package(catkin REQUIRED COMPONENTS 
    roscpp rospy std_msgs
    )

find_package(Qt5 REQUIRED Core Widgets  Concurrent )
set(QT_LIBRARIES Qt5::Widgets  Qt5::Concurrent )


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(launch_node src/main.cpp)
target_link_libraries(launch_node ${catkin_LIBRARIES} ${QT_LIBRARIES})
