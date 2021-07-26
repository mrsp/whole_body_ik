#ifndef __NAOWBC_H__
#define __NAOWBC_H__
#include <whole_body_ik/pin_wrapper.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <whole_body_ik_msgs/HumanoidAction.h>
#include <eigen3/Eigen/Dense>
#include <actionlib/server/simple_action_server.h>

using std::cerr;
using std::endl;
using std::string;

using namespace Eigen;


class nao_wbc
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;

    bool firstJointCb = true;
    bool joint_inc = false;
    ros::Subscriber joint_state_sub, odom_sub;
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame, lhand_frame, rhand_frame, head_frame, joint_cmd_topic;
    double joint_freq;
    ros::Publisher cmd_pub;
    whole_body_ik_msgs::HumanoidResult result_;
    whole_body_ik_msgs::HumanoidFeedback feedback_;
    
    std::vector<linearTask> ltaskVec;
    std::vector<angularTask> ataskVec;
    std::vector<dofTask> dtaskVec;
    humanoidTaskData htd;

    Eigen::VectorXd q,dq, qd, dqd;
    bool LSS, RSS, DS;
    double dt;
    Vector3d pwb, vwb, omegawb;
    Quaterniond qwb;

    pin_wrapper *pin;

    void init();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ~nao_wbc();
    nao_wbc(ros::NodeHandle nh_);
    void joint_stateCb(const sensor_msgs::JointStateConstPtr &msg);
    void odomCb(const nav_msgs::OdometryConstPtr &msg);
    void run();
    actionlib::SimpleActionServer<whole_body_ik_msgs::HumanoidAction> *as_; 
    void controlCb(const whole_body_ik_msgs::HumanoidGoalConstPtr &msg);

void swapQuatWXYZ(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[6];
  input_[4] = tmp[3];
  input_[5] = tmp[4];
  input_[6] = tmp[5];
}
void swapQuatXYZW(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[4];
  input_[4] = tmp[5];
  input_[5] = tmp[6];
  input_[6] = tmp[3];
}



};
#endif
