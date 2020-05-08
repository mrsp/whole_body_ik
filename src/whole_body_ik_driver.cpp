#include <iostream>
#include <whole_body_ik/pin_wrapper.h>
#include <whole_body_ik/LeakyIntegrator.h>
#include <ros/ros.h>
using std::cerr;
using std::endl;
using std::string;

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
    n_p.param<std::string>("modelname", modelname, "/home/master/Desktop/nao_walk_ws/src/whole_body_ik/share/urdf/nao.urdf");
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("lfoot", lfoot_frame, "l_ankle");
    n_p.param<std::string>("rfoot", rfoot_frame, "r_ankle");
    n_p.param<double>("joint_freq", joint_freq, 100.0);


    //Initialize the Pinocchio Wrapper
    pin_wrapper *pin = new pin_wrapper(modelname, false);
    pin->setdt(1.0 / joint_freq);

    //NAO Joints
    std::string joint_names[26] = { "HeadYaw",
                                    "HeadPitch",
                                    "LShoulderPitch",
                                    "LShoulderRoll",
                                    "LElbowYaw",
                                    "LElbowRoll",
                                    "LWristYaw",
                                    "LHand",
                                    "LHipYawPitch",
                                    "LHipRoll",
                                    "LHipPitch",
                                    "LKneePitch",
                                    "LAnklePitch",
                                    "LAnkleRoll",
                                    "RHipYawPitch",
                                    "RHipRoll",
                                    "RHipPitch",
                                    "RKneePitch",
                                    "RAnklePitch",
                                    "RAnkleRoll",
                                    "RShoulderPitch",
                                    "RShoulderRoll",
                                    "RElbowYaw",
                                    "RElbowRoll",
                                    "RWristYaw",
                                    "RHand"};

    double joint_positions[26] = { -0.0123138427734375, -0.013848066329956055, 1.501744031906128, 0.2929520606994629, -1.362234115600586, -0.31136012077331543,
                                   0.02603602409362793, 0.24239999055862427, 4.1961669921875e-05, -0.029103994369506836, -0.4540219306945801, 1.2271580696105957,
                                   -0.7731781005859375, 0.03072190284729004, 0.0, 0.0015759468078613281, -0.5292720794677734, 1.2533202171325684,
                                   -0.7194039821624756, 0.0015759468078613281, 1.5877318382263184, -0.3298518657684326, 1.348344087600708, 0.3528618812561035,
                                   -0.02611994743347168, 0.2476000189781189 };
    double joint_velocities[26] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
   



    int i=0;
    std::map<std::string, double> joint_state_pos_map, joint_state_vel_map;
    while (i < 26)
    {
        joint_state_pos_map[joint_names[i]] = joint_positions[i];
        joint_state_vel_map[joint_names[i]] = joint_velocities[i];
        i++;
    }

    //Update Joint_states in Pinocchio
    pin->updateJointConfig(joint_state_pos_map, joint_state_vel_map);
    LeakyIntegrator li[26];
    while(i<26)
    {
        li[i].rate(10.0);
        li[i].setInitialState(pin->qq[i]);
        i++;
    }


    //Define Tasks
    //Left Foot Swing Task
    Eigen::Vector3d vdes0 = Eigen::Vector3d(10, 10, 10);
    double weight0 = 1;
    double gain0 = 0.5;
    int task_type0 = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    pin->setTask(lfoot_frame, task_type0,vdes0, weight0, gain0);

    //CoM task
    int task_type1 = 2; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    Eigen::Vector3d vdes1 = Eigen::Vector3d(0.01, 10, 0.01);
    double weight1 = 1;
    double gain1 = 0.5;
    pin->setTask("CoM", task_type1, vdes1,  weight1,  gain1);

    //Right Foot Support task
    int task_type2 = 0; //0 for linear velocity/ 1 for angular velocity / 2 for CoM
    Eigen::Vector3d vdes2 = Eigen::Vector3d(0.0, 0.0, 0.0);
    double weight2 = 100.0;
    double gain2 = 0.5;
    pin->setTask(rfoot_frame, task_type2, vdes2,  weight2,  gain2);


    Eigen::VectorXd qdotd = pin->inverseKinematics();
    Eigen::VectorXd qd = qdotd * 0;
    i=0;
    while(i<26)
    {
        li[i].add(qdotd(i), 1.0/joint_freq);
        qd(i) = li[i].eval();
        i++;
    }
    std::cout<<"Desired Joint State "<<std::endl;
    std::cout<<qd<<std::endl;

    
    //Done here
    ROS_INFO("Quitting... ");
    delete pin;
    return 0;
}