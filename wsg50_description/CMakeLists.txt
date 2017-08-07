cmake_minimum_required(VERSION 2.8.3)
project(wsg50_description)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

###########
## Build ##
###########


#############
## Install ##
#############

install(DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# roslaunch_add_file_check(test/roslaunch_test.xml)
