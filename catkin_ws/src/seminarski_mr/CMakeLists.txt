cmake_minimum_required(VERSION 2.8.3)
project(seminarski_mr)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Find OpenCV package
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

##include_directories(include ${catkin_INCLUDE_DIRS} /home/tarik/opencv/include) 

##add_executable(talker src/talker.cpp)
##target_link_libraries(talker ${catkin_LIBRARIES})
##add_dependencies(talker beginner_tutorials_generate_messages_cpp)

##add_executable(listener src/listener.cpp)
##target_link_libraries(listener ${catkin_LIBRARIES})
##add_dependencies(listener beginner_tutorials_generate_messages_cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})

add_executable(listener1 src/listener1.cpp)
target_link_libraries(listener1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(listener1 ${catkin_EXPORTED_TARGETS})


