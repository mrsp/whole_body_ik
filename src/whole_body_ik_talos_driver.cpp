#include <whole_body_ik/talos.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_ik_talos_driver");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }

    talos* tl;
    tl = new talos(n);
    tl->run();
    ros::spin();
    //Done here
    ROS_INFO("Quitting... ");
    delete tl;
    return 0;
}
