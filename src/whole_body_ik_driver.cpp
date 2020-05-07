#include <iostream>
#include <whole_body_ik/pin_wrapper.h>
#include <ros/ros.h>
using std::string;
using std::cerr;
using std::endl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "whole_body_ik");
    ros::NodeHandle n;
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }
    ros::NodeHandle n_p("~");
    std::string modelname, base_link_frame, lfoot_frame, rfoot_frame;
    double joint_freq;
    n_p.param<std::string>("modelname", modelname, "/home/master/Desktop/nao_walk_ws/src/whole_body_ik/share/urdf/nao.urdf");
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("lfoot", lfoot_frame, "l_ankle");
    n_p.param<std::string>("rfoot", rfoot_frame, "r_ankle");
    n_p.param<double>("joint_freq", joint_freq, 100.0);

    pin_wrapper* pin =  new pin_wrapper(modelname, false);
    pin->setdt(1.0/joint_freq);
    std::cout<<pin->inverseKinematics()<<std::endl;
    
    //Done here
    ROS_INFO( "Quitting... " );

    return 0;
}