#include <iostream>
#include <whole_body_ik/pin_wrapper.h>
#include <whole_body_ik/LeakyIntegrator.h>
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
    std::cout<<"Joint Callback"<<std::endl;
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
    ros::init(argc, argv, "whole_body_ik");
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
    n_p.param<std::string>("gripper_left_base_link", lhand_frame, "left_sole_link");
    n_p.param<std::string>("gripper_right_base_link", rhand_frame, "right_sole_link");
    n_p.param<std::string>("rgbd_link", head_frame, "rgbd_link");
    n_p.param<double>("joint_freq", joint_freq, 1.0);


    //Initialize the Pinocchio Wrapper
    pin_wrapper *pin = new pin_wrapper(modelname, false);
    pin->setdt(1.0 / joint_freq);

    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, joint_stateCb);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_head("/head_controller/follow_joint_trajectory",true);
    ac_head.waitForServer();

    static ros::Rate rate(0.5*joint_freq);
    while(ros::ok())
    {
        if(joint_inc)
        {
            joint_inc = false;
            break;
        }
        rate.sleep();
        ros::spinOnce();

    }
    

    //Update Joint_states in Pinocchio
    pin->updateJointConfig(joint_names,joint_positions,joint_velocities);
    
    //pin->getJointData(joint_names,joint_positions,joint_velocities);
    
    
    int i=0;
    LeakyIntegrator li[joint_names.size()];
    while(i<joint_names.size())
    {
        li[i].rate(10.0);
        li[i].setInitialState(joint_positions[i]);
        i++;
    }


    //Define Tasks
    //Left Foot Swing Task
    Eigen::Vector3d vdes;
    double weight, gain;
    int task_type;

    vdes.setZero();
    weight = 100.0;
    gain = 0.5;
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type,vdes, weight, gain);

    vdes.setZero();
    weight = 100.0;
    gain = 0.5;
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type,vdes, weight, gain);

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
    weight = 100.0;
    gain = 0.5;
    pin->setTask(rfoot_frame, task_type, vdes,  weight,  gain);

    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 100.0;
    gain = 0.5;
    pin->setTask(rfoot_frame, task_type, vdes,  weight,  gain);

    //Head
    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes = Eigen::Vector3d(-0.0, -0.0, -2.5);
    weight = 1000;
    gain = 0.5;
    pin->setTask(head_frame, task_type, vdes,  weight,  gain);

    



    //Right Hand 
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(rhand_frame, task_type, vdes,  weight,  gain);

    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(rhand_frame, task_type, vdes,  weight,  gain);


    //Left Hand 
    task_type = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(lhand_frame, task_type, vdes,  weight,  gain);

    task_type = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    vdes.setZero();
    weight = 0.1;
    gain = 0.5;
    pin->setTask(lhand_frame, task_type, vdes,  weight,  gain);



    std::vector<double> desired_joint_velocities;

    pin->inverseKinematics();
    pin->getDesiredJointData(joint_names,desired_joint_velocities);

    std::vector<double> desired_joint_states;
    desired_joint_states.resize(desired_joint_velocities.size());
    i=0;
    while(i<desired_joint_velocities.size())
    {
        li[i].add(desired_joint_velocities[i], 1.0/joint_freq);
        desired_joint_states[i]=li[i].eval();
        i++;
    }


    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.goal_time_tolerance = ros::Duration(0.005);
    head_goal.trajectory.header.stamp = ros::Time::now();
    head_goal.trajectory.joint_names = {"head_1_joint","head_2_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(2);
    point.positions[0]= pin->getQdotd("head_1_joint")/joint_freq + pin->getQq("head_1_joint");
    point.positions[1]= pin->getQdotd("head_2_joint")/joint_freq + pin->getQq("head_2_joint");

        std::cout<<"Desired Joint State "<<point.positions[0]<<" "<<point.positions[1]<<std::endl;
        std::cout<<"Desired Joint Vel "<<pin->getQdotd("head_1_joint")<<" "<<pin->getQdotd("head_2_joint")<<std::endl;

    //point.positions[0]= 0;
    //point.positions[1]= 0;
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0]=point;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(0.05);

    //Sends the goal to the action server.
    ac_head.sendGoal(head_goal);
   
    
    //Done here
    ROS_INFO("Quitting... ");
    delete pin;
    return 0;
}
