cmake_minimum_required(VERSION 2.8.3)
project(gpioPWM)
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp rospy std_msgs std_srvs genmsg
	message_generation
)
find_package(Eigen REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)
add_message_files(
	FILES gpioPWM.msg
)

generate_messages(
	DEPENDENCIES std_msgs
)

#add_executable(pioneer_path src/circle.cpp)
#target_link_libraries(pioneer_path ${catkin_LIBRARIES})

#add_executable(pioneer_path src/main.cpp)
#target_link_libraries(pioneer_path ${catkin_LIBRARIES})

#include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(include include)
#include_directories(${Eigen_INCLUDE_DIRS})

#add_executable(pioneer_path_gen src/path_generator/main.cpp)
#target_link_libraries(pioneer_path_gen ${catkin_LIBRARIES})
