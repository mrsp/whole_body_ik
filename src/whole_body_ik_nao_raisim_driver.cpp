#include <whole_body_ik/nao_wbc.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_ik_nao_raisim_driver");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }

    nao_wbc* NAO;
    NAO = new nao_wbc(n);
    NAO->run();
    ros::spin();
    //Done here
    ROS_INFO("Quitting... ");
    delete NAO;
    return 0;
}
