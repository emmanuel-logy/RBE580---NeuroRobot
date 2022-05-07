cmake_minimum_required(VERSION 3.0.0)
project(NeuroRobot_MotionPlanning)

# add_definitions() #-DDEBUG -DVIEW_TABLE
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")		# change it to "Release" when needed

################################################
## Find all dependencies ##
################################################
find_package (Eigen3 3.3.0 REQUIRED)



###########
## Build ##
###########

########## Building kinematics library ##########
include_directories(Kinematics 
     		     Robots)
set(SOURCES_NEUROKINEMATICS Kinematics/NeuroKinematics/NeuroKinematics.cpp
			     Robots/NeuroRobot/NeuroRobot.cpp)	
message(${SOURCES_NEUROKINEMATICS})		     
add_library(NeuroKinematics ${SOURCES_NEUROKINEMATICS})

########## Building kinematics exe ##########
add_executable(test_IK main.cpp)
target_link_libraries(test_IK NeuroKinematics)

