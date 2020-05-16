#include <iostream>
#include <whole_body_ik/pin_wrapper.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using std::cerr;
using std::endl;
using std::string;

std::vector<std::string> joint_names;
std::vector<double> joint_positions;
std::vector<double> joint_velocities;
bool firstJointCb = true;
bool joint_inc = false;

void joint_stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_inc = true;
    std::cout << "Joint Callback" << std::endl;
    if (firstJointCb)
    {
        joint_names.resize(msg->name.size());
        joint_positions.resize(msg->position.size());
        joint_velocities.resize(msg->velocity.size());
        firstJointCb = false;
    }
    joint_positions = msg->position;
    joint_velocities = msg->velocity;
    joint_names = msg->name;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_ik_talos_driver");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }
    ros::NodeHandle n_p("~");
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame, lhand_frame, rhand_frame, head_frame;
    double joint_freq;
    n_p.param<std::string>("modelname", modelname, "/home/master/talos_walk_ws/src/whole_body_ik/share/urdf/talos_full_v2.urdf");
    //n_p.param<std::string>("modelname", modelname, "/home/tavu/catkin_ws/src/whole_body_ik/share/urdf/nao.urdf");
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("leg_left_6_link", lfoot_frame, "leg_left_6_link");
    n_p.param<std::string>("leg_right_6_link", rfoot_frame, "leg_right_6_link");
    n_p.param<std::string>("arm_left_7_link", lhand_frame, "arm_left_7_link");
    n_p.param<std::string>("arm_right_7_link", rhand_frame, "arm_right_7_link");
    n_p.param<std::string>("rgbd_link", head_frame, "rgbd_link");
    n_p.param<double>("joint_freq", joint_freq, 1.0);

    //Initialize the Pinocchio Wrapper
    pin_wrapper *pin = new pin_wrapper(modelname, false);

    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 10, joint_stateCb);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_head("/head_controller/follow_joint_trajectory", true);
    ac_head.waitForServer();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_leftleg("/left_leg_controller/follow_joint_trajectory", true);
    ac_leftleg.waitForServer();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_rightleg("/right_leg_controller/follow_joint_trajectory", true);
    ac_rightleg.waitForServer();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_leftarm("/left_arm_controller/follow_joint_trajectory", true);
    ac_leftarm.waitForServer();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_rightarm("/right_arm_controller/follow_joint_trajectory", true);
    ac_rightarm.waitForServer();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_torso("/torso_controller/follow_joint_trajectory", true);
    ac_torso.waitForServer();

    static ros::Rate rate(2.0 * joint_freq);
    while (ros::ok())
    {
        if (joint_inc)
        {
            joint_inc = false;
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }

    //Update Joint_states in Pinocchio
    pin->updateJointConfig(joint_names, joint_positions, joint_velocities);
    pin->setPoistionControl(0);

    //Define Tasks
    Eigen::Vector3d vdes;
    double weight, gain;
    int task_type;

    //Left Foot Task
    vdes.setZero();
    weight = 1000.0;
    gain = 0.5;
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type, vdes, weight, gain);

    vdes.setZero();
    weight = 1000.0;
    gain = 0.5;
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type, vdes, weight, gain);

    /*
    //CoM task
    task_type = 2; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 5;
    gain = 0.5;
    pin->setTask("CoM", task_type, vdes,  weight,  gain);
*/
    //Right Foot Support task
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 1000.0;
    gain = 0.5;
    pin->setTask(rfoot_frame, task_type, vdes, weight, gain);
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 1000.0;
    gain = 0.5;
    pin->setTask(rfoot_frame, task_type, vdes, weight, gain);
    //Head
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes = Eigen::Vector3d(-0.0, -0.0, -2.5);
    weight = 200;
    gain = 0.5;
    pin->setTask(head_frame, task_type, vdes, weight, gain);
    //Head
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes = Eigen::Vector3d(0.0, 0.0, 0.0);
    weight = 100;
    gain = 0.5;
    pin->setTask(base_link_frame, task_type, vdes, weight, gain);
    //Right Hand
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 100.0;
    gain = 0.5;
    pin->setTask(rhand_frame, task_type, vdes, weight, gain);

    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(rhand_frame, task_type, vdes, weight, gain);

    //Left Hand
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 100;
    gain = 0.5;
    pin->setTask(lhand_frame, task_type, vdes, weight, gain);

    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(lhand_frame, task_type, vdes, weight, gain);

    double dt = 1.0 / joint_freq;
    pin->inverseKinematics(dt);

    pin->printDesiredJointData();

    //Send Desired Joints to Talos
    //Head Joint Controller
    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    head_goal.trajectory.header.stamp = ros::Time::now();
    head_goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
    trajectory_msgs::JointTrajectoryPoint head_point;
    head_point.positions.resize(2);
    head_point.positions[0] = pin->getQd("head_1_joint");
    head_point.positions[1] = pin->getQd("head_2_joint");
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0] = head_point;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_head.sendGoal(head_goal);
    //Torso Joint Controller
    control_msgs::FollowJointTrajectoryGoal torso_goal;
    torso_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    torso_goal.trajectory.header.stamp = ros::Time::now();
    torso_goal.trajectory.joint_names = {"torso_1_joint", "torso_2_joint"};
    trajectory_msgs::JointTrajectoryPoint torso_point;
    torso_point.positions.resize(2);
    torso_point.positions[0] = pin->getQd("torso_1_joint");
    torso_point.positions[1] = pin->getQd("torso_2_joint");
    torso_goal.trajectory.points.resize(1);
    torso_goal.trajectory.points[0] = torso_point;
    torso_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_torso.sendGoal(torso_goal);
    //Left Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal larm_goal;
    larm_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    larm_goal.trajectory.header.stamp = ros::Time::now();
    larm_goal.trajectory.joint_names = {"arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"};
    trajectory_msgs::JointTrajectoryPoint larm_point;
    larm_point.positions.resize(7);
    larm_point.positions[0] = pin->getQd("arm_left_1_joint");
    larm_point.positions[1] = pin->getQd("arm_left_2_joint");
    larm_point.positions[2] = pin->getQd("arm_left_3_joint");
    larm_point.positions[3] = pin->getQd("arm_left_4_joint");
    larm_point.positions[4] = pin->getQd("arm_left_5_joint");
    larm_point.positions[5] = pin->getQd("arm_left_6_joint");
    larm_point.positions[6] = pin->getQd("arm_left_7_joint");
    larm_goal.trajectory.points.resize(1);
    larm_goal.trajectory.points[0] = larm_point;
    larm_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_leftarm.sendGoal(larm_goal);
    //Right Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal rarm_goal;
    rarm_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    rarm_goal.trajectory.header.stamp = ros::Time::now();
    rarm_goal.trajectory.joint_names = {"arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"};
    trajectory_msgs::JointTrajectoryPoint rarm_point;
    rarm_point.positions.resize(7);
    rarm_point.positions[0] = pin->getQd("arm_right_1_joint");
    rarm_point.positions[1] = pin->getQd("arm_right_2_joint");
    rarm_point.positions[2] = pin->getQd("arm_right_3_joint");
    rarm_point.positions[3] = pin->getQd("arm_right_4_joint");
    rarm_point.positions[4] = pin->getQd("arm_right_5_joint");
    rarm_point.positions[5] = pin->getQd("arm_right_6_joint");
    rarm_point.positions[6] = pin->getQd("arm_right_7_joint");
    rarm_goal.trajectory.points.resize(1);
    rarm_goal.trajectory.points[0] = rarm_point;
    rarm_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_rightarm.sendGoal(rarm_goal);
    //Left Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal lleg_goal;
    lleg_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    lleg_goal.trajectory.header.stamp = ros::Time::now();
    lleg_goal.trajectory.joint_names = {"leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint"};
    trajectory_msgs::JointTrajectoryPoint lleg_point;
    lleg_point.positions.resize(6);
    lleg_point.positions[0] = pin->getQd("leg_left_1_joint");
    lleg_point.positions[1] = pin->getQd("leg_left_2_joint");
    lleg_point.positions[2] = pin->getQd("leg_left_3_joint");
    lleg_point.positions[3] = pin->getQd("leg_left_4_joint");
    lleg_point.positions[4] = pin->getQd("leg_left_5_joint");
    lleg_point.positions[5] = pin->getQd("leg_left_6_joint");
    lleg_goal.trajectory.points.resize(1);
    lleg_goal.trajectory.points[0] = lleg_point;
    lleg_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_leftleg.sendGoal(lleg_goal);
    //Right Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal rleg_goal;
    rleg_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    rleg_goal.trajectory.header.stamp = ros::Time::now();
    rleg_goal.trajectory.joint_names = {"leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint"};
    trajectory_msgs::JointTrajectoryPoint rleg_point;
    rleg_point.positions.resize(6);
    rleg_point.positions[0] = pin->getQdotd("leg_right_1_joint");
    rleg_point.positions[1] = pin->getQdotd("leg_right_2_joint");
    rleg_point.positions[2] = pin->getQdotd("leg_right_3_joint");
    rleg_point.positions[3] = pin->getQdotd("leg_right_4_joint");
    rleg_point.positions[4] = pin->getQdotd("leg_right_5_joint");
    rleg_point.positions[5] = pin->getQdotd("leg_right_6_joint");
    rleg_goal.trajectory.points.resize(1);
    rleg_goal.trajectory.points[0] = rleg_point;
    rleg_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_rightleg.sendGoal(rleg_goal);

    //Done here
    ROS_INFO("Quitting... ");
    delete pin;
    return 0;
}