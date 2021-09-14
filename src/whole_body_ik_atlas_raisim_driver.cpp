#include <whole_body_ik/atlas_wbc.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_ik_atlas_raisim_driver");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }

    atlas_wbc* ATLAS;
    ATLAS = new atlas_wbc(n);
    ATLAS->run();
    ros::spin();
    //Done here
    ROS_INFO("Quitting... ");
    delete ATLAS;
    return 0;
}
