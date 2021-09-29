cmake_minimum_required(VERSION 2.8.3)
project(whole_body_ik)

add_compile_options(-std=c++11)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  actionlib
  whole_body_ik_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

find_package(qpmad)

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whole_body_ik
  CATKIN_DEPENDS roscpp sensor_msgs actionlib whole_body_ik_msgs 
  DEPENDS EIGEN3 PINOCCHIO qpmad
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${qpmad_INCLUDE_DIRS}
)

link_directories(${PINOCCHIO_LIBRARY_DIRS})

SET(humanoid_cpp_src
    src/humanoid_wbc.cpp
    src/pin_wrapper.cpp
)

add_executable(whole_body_ik_node_humanoid  src/whole_body_ik_humanoid_driver.cpp  ${humanoid_cpp_src})
target_link_libraries(whole_body_ik_node_humanoid ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
target_compile_definitions(whole_body_ik_node_humanoid PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
add_dependencies(whole_body_ik_node_humanoid  ${catkin_EXPORTED_TARGETS})

add_library(whole_body_ik ${humanoid_cpp_src})
target_link_libraries(whole_body_ik ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
target_compile_definitions(whole_body_ik PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
add_dependencies(whole_body_ik  ${catkin_EXPORTED_TARGETS})