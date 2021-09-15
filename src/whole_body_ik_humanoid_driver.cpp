#include <whole_body_ik/humanoid_wbc.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_ik_humanoid_driver");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }

    humanoid_wbc* robot;
    robot = new humanoid_wbc(n);
    robot->run();
    ros::spin();
    //Done here
    ROS_INFO("Quitting... ");
    delete robot;
    return 0;
}
