cmake_minimum_required(VERSION 2.8.3)
project(whole_body_ik)

add_compile_options(-std=c++11)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  control_msgs
  actionlib
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(qpmad)



find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whole_body_ik
  CATKIN_DEPENDS roscpp sensor_msgs control_msgs actionlib
  DEPENDS Eigen3 pinocchio qpmad
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${qpmad_INCLUDE_DIRS}
)

link_directories(${PINOCCHIO_LIBRARY_DIRS})

## CPP source files
SET(cpp_src
    src/whole_body_ik_talos_driver.cpp 
    src/LeakyIntegrator.cpp
    src/pin_wrapper.cpp
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(whole_body_ik ${cpp_src})


target_link_libraries(whole_body_ik ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
target_compile_definitions(whole_body_ik PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
add_dependencies(whole_body_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
