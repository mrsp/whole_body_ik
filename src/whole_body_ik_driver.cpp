#include <iostream>
#include <whole_body_ik/pin_wrapper.h>
#include <whole_body_ik/LeakyIntegrator.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
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
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame;
    double joint_freq;
    n_p.param<std::string>("modelname", modelname, "/home/master/talos_walk_ws/src/whole_body_ik/share/urdf/talos_full_v2.urdf");
    //n_p.param<std::string>("modelname", modelname, "/home/tavu/catkin_ws/src/whole_body_ik/share/urdf/nao.urdf");
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("lfoot", lfoot_frame, "left_sole_link");
    n_p.param<std::string>("rfoot", rfoot_frame, "right_sole_link");
    n_p.param<double>("joint_freq", joint_freq, 5000.0);


    //Initialize the Pinocchio Wrapper
    pin_wrapper *pin = new pin_wrapper(modelname, false);
    pin->setdt(1.0 / joint_freq);

    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, joint_stateCb);
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
    Eigen::Vector3d vdes0 = Eigen::Vector3d(0, 0, 0);
    double weight0 = 100.0;
    double gain0 = 0.5;
    int task_type0 = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type0,vdes0, weight0, gain0);

    //CoM task
    int task_type1 = 2; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    Eigen::Vector3d vdes1 = Eigen::Vector3d(0.00, 0, 0.00);
    double weight1 = 1;
    double gain1 = 0.5;
    pin->setTask("CoM", task_type1, vdes1,  weight1,  gain1);

    //Right Foot Support task
    int task_type2 = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    Eigen::Vector3d vdes2 = Eigen::Vector3d(0.0, 0.0, 0.0);
    double weight2 = 100.0;
    double gain2 = 0.5;
    pin->setTask(rfoot_frame, task_type2, vdes2,  weight2,  gain2);

    //Head
    int task_type3 = 1; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    Eigen::Vector3d vdes3 = Eigen::Vector3d(0.1, 0.1, 0.0);
    double weight3 = 0.1;
    double gain3 = 0.5;
    pin->setTask(rfoot_frame, task_type3, vdes3,  weight3,  gain3);



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
    std::cout<<"Desired Joint State "<<i<<std::endl;
    //std::cout<<desired_joint_states<<std::endl;
    int j = 0;
    while(j<joint_names.size())
    {
        std::cout<<"join pos "<<desired_joint_states[j]<<std::endl;
        std::cout<<"join vel "<<desired_joint_velocities[j]<<std::endl;

        j++;
    }
   
    
    //Done here
    ROS_INFO("Quitting... ");
    delete pin;
    return 0;
}
