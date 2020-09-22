#ifndef __TALOSWBC_H__
#define __TALOSWBC_H__
#include <whole_body_ik/pin_wrapper.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <whole_body_ik_msgs/HumanoidAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>

using std::cerr;
using std::endl;
using std::string;

using namespace Eigen;


class talos
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    bool firstJointCb = true;
    bool joint_inc = false;
    ros::Subscriber joint_state_sub;
    pin_wrapper *pin;
    whole_body_ik_msgs::HumanoidResult result_;
    whole_body_ik_msgs::HumanoidFeedback feedback_;
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame, lhand_frame, rhand_frame, head_frame;
    double joint_freq;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac_head, *ac_leftleg,*ac_rightleg,*ac_leftarm,*ac_rightarm,*ac_torso;
    actionlib::SimpleActionServer<whole_body_ik_msgs::HumanoidAction> *as_; 
    ~talos();
    talos(ros::NodeHandle nh_);
    void controlCb(const whole_body_ik_msgs::HumanoidGoalConstPtr &msg);
    void joint_stateCb(const sensor_msgs::JointStateConstPtr &msg);
    void run();
    void walking();
};
#endif
