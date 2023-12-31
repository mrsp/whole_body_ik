# README
Whole-body Task Space Inverse Kinematics for Humanoid/Bipedal legged robots. The code is open-source (BSD License). Please note that this work is an on-going research and thus some parts are not fully developed yet. Furthermore, the code will be subject to changes in the future which could include greater re-factoring.


# Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

## Prerequisites
* Ubuntu 16.04 and later
* ROS kinetic and later
* Eigen 3.2.0 and later
* Pinocchio: https://github.com/stack-of-tasks/pinocchio
* qpmad: https://github.com/asherikov/qpmad

## Installing
* `git clone https://github.com/mrsp/whole_body_ik_msgs.git`
* `git clone https://github.com/mrsp/whole_body_ik.git`
* `catkin_make -DCMAKE_BUILD_TYPE=Release` 
* If you are using catkin tools run: `catkin build  --cmake-args -DCMAKE_BUILD_TYPE=Release` 

## ROS Examples
### Launch on your Robot 
* Specify topics on `config/humanoid_wbc_params.yaml`
* `roslaunch whole_body_ik humanoid_wbc.launch`

### Whole-body Control for NAO and Atlas humanoid robots
[![YouTube Link](img/atlasWBC.png)  ](https://www.youtube.com/watch?v=NjRICIC1yZE)
[![YouTube Link](img/atlasWALK.png)  ](https://www.youtube.com/watch?v=Q8YF8RGbtac)

## License
[BSD](LICENSE) 

