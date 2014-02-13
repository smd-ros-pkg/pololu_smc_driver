cmake_minimum_required(VERSION 2.8.3)
project(pololu_smc_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure nodelet pluginlib roscpp std_msgs trajectory_msgs std_srvs)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

generate_dynamic_reconfigure_options(
  cfg/SMCDriver.py
  cfg/SMCChannel.py
  cfg/SMCLimits.py
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES smc_driver
   CATKIN_DEPENDS dynamic_reconfigure nodelet pluginlib roscpp std_msgs trajectory_msgs std_srvs
#   DEPENDS libusb-1.0
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB1_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(smc
  src/smc.c
)
add_library(smc_driver
  src/smc_driver.cpp
)
add_library(smc_driver_nodelet
  src/smc_driver_nodelet.cpp
)

## Declare a cpp executable
add_executable(smc_list src/smc_list.c)
add_executable(smc_driver_node src/smc_driver_node.cpp)

## Add dependencies to the executable
add_dependencies(smc_driver ${PROJECT_NAME}_gencfg)
add_dependencies(smc_driver_node smc_driver)
add_dependencies(smc_driver_nodelet smc_driver)

## Specify libraries to link a library or executable target against
target_link_libraries(smc_list
  smc
  ${LIBUSB1_LIBRARIES}
)
target_link_libraries(smc_driver
  smc
  ${LIBUSB1_LIBRARIES}
)
target_link_libraries(smc_driver_node
  smc_driver
  ${catkin_LIBRARIES}
)
target_link_libraries(smc_driver_nodelet
  smc_driver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS smc smc_list smc_driver smc_driver_node smc_driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/pololu_smc_driver/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pololu_smc_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
