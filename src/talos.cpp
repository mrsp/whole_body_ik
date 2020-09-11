#include <whole_body_ik/talos.h>

talos::talos(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    n_p.param<std::string>("modelname", modelname, "/home/master/catkin_ws/src/whole_body_ik/share/urdf/talos_full_v2.urdf");
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("leg_left_6_link", lfoot_frame, "leg_left_6_link");
    n_p.param<std::string>("leg_right_6_link", rfoot_frame, "leg_right_6_link");
    n_p.param<std::string>("arm_left_7_link", lhand_frame, "arm_left_7_link");
    n_p.param<std::string>("arm_right_7_link", rhand_frame, "arm_right_7_link");
    n_p.param<std::string>("rgbd_link", head_frame, "rgbd_link");
    n_p.param<double>("joint_freq", joint_freq, 50.0);

    //Initialize the Pinocchio Wrapper
     pin = new pin_wrapper(modelname, false);

    joint_state_sub = nh.subscribe("/joint_states", 10, &talos::joint_stateCb, this);

    ac_head = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/head_controller/follow_joint_trajectory", true);
    ac_head->waitForServer();

    ac_leftleg =  new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/left_leg_controller/follow_joint_trajectory", true);
    ac_leftleg->waitForServer();

    ac_rightleg = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/right_leg_controller/follow_joint_trajectory", true);
    ac_rightleg->waitForServer();

    ac_leftarm = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/left_arm_controller/follow_joint_trajectory", true);
    ac_leftarm->waitForServer();

    ac_rightarm = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/right_arm_controller/follow_joint_trajectory", true);
    ac_rightarm->waitForServer();

    ac_torso = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/torso_controller/follow_joint_trajectory", true);
    ac_torso->waitForServer();
    std::cout<<"Connected to Talos Controllers"<<std::endl;
    as_ = new actionlib::SimpleActionServer<whole_body_ik_msgs::HumanoidAction>(nh, "/talos/whole_body_control", boost::bind(&talos::controlCb, this, _1), false);
    as_->start();
    std::cout<<"Talos Whole Body Control Server Online"<<std::endl;
    ros::Duration(0.5).sleep();
    walking();
}
void talos::joint_stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_inc = true;
    if(firstJointCb)
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
void talos::run()
{
    static ros::Rate rate(2.0 * joint_freq);
    while (ros::ok())
    {
        if (joint_inc)
        {
            //Update Joint_states in Pinocchio
            pin->updateJointConfig(joint_names, joint_positions, joint_velocities);
            if(!pin->initialized)
            {
                pin->setPoistionControl(0);
                pin->initialized = true;
                std::cout<<pin->comPosition()<<std::endl;
            }
            joint_inc = false;
        }
        rate.sleep();
        ros::spinOnce();
    }

}

void talos::walking()
{



    double dt = 5.0;
     //Send Desired Joints to Talos
    //Head Joint Controller
    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.goal_time_tolerance = ros::Duration(2.0*dt);
    head_goal.trajectory.header.stamp = ros::Time::now();
    head_goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
    trajectory_msgs::JointTrajectoryPoint head_point;
    head_point.positions.resize(2);
    head_point.positions[0] =  0.00041801880549030557;
    head_point.positions[1] =  -8.981051937695383e-07;
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0] = head_point;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_head->sendGoal(head_goal);
    //Torso Joint Controller
    control_msgs::FollowJointTrajectoryGoal torso_goal;
    torso_goal.goal_time_tolerance = ros::Duration(2.0*dt);
    torso_goal.trajectory.header.stamp = ros::Time::now();
    torso_goal.trajectory.joint_names = {"torso_1_joint", "torso_2_joint"};
    trajectory_msgs::JointTrajectoryPoint torso_point;
    torso_point.positions.resize(2);
    torso_point.positions[0] = 9.630276567307305e-07;
    torso_point.positions[1] = 0.00025052828388183457;
    torso_goal.trajectory.points.resize(1);
    torso_goal.trajectory.points[0] = torso_point;
    torso_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_torso->sendGoal(torso_goal);
    //Left Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal larm_goal;
    larm_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    larm_goal.trajectory.header.stamp = ros::Time::now();
    larm_goal.trajectory.joint_names = {"arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"};
    trajectory_msgs::JointTrajectoryPoint larm_point;
    larm_point.positions.resize(7);
    larm_point.positions[0] = 0.299975229388715;
    larm_point.positions[1] = 0.39996282399174987;
    larm_point.positions[2] = -0.5000333212143211;
    larm_point.positions[3] = -1.5000561487005104;
    larm_point.positions[4] = -0.004710635566556931;
    larm_point.positions[5] = 0.0008074396976889275;
    larm_point.positions[6] = 0.00033309628302102823;
    larm_goal.trajectory.points.resize(1);
    larm_goal.trajectory.points[0] = larm_point;
    larm_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_leftarm->sendGoal(larm_goal);
    //Right Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal rarm_goal;
    rarm_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    rarm_goal.trajectory.header.stamp = ros::Time::now();
    rarm_goal.trajectory.joint_names = {"arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"};
    trajectory_msgs::JointTrajectoryPoint rarm_point;
    rarm_point.positions.resize(7);
    rarm_point.positions[0] = -0.2999902314497742;
    rarm_point.positions[1] = -0.399969195815034;
    rarm_point.positions[2] = 0.5000332008362696;
    rarm_point.positions[3] = -1.5000893794782577;
    rarm_point.positions[4] =  0.004731219404664699;
    rarm_point.positions[5] = -0.0005641853878053382;
    rarm_point.positions[6] =  0.0004513709144031708;
    rarm_goal.trajectory.points.resize(1);
    rarm_goal.trajectory.points[0] = rarm_point;
    rarm_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_rightarm->sendGoal(rarm_goal);
    //Left Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal lleg_goal;
    lleg_goal.goal_time_tolerance = ros::Duration(2.0 *dt);
    lleg_goal.trajectory.header.stamp = ros::Time::now();
    lleg_goal.trajectory.joint_names = {"leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint"};
    trajectory_msgs::JointTrajectoryPoint lleg_point;
    lleg_point.positions.resize(6);
    lleg_point.positions[0] =  -8.124331024372822e-05;
    lleg_point.positions[1] = -0.000831040487059731;
    lleg_point.positions[2] = -0.5194120606620993;
    lleg_point.positions[3] = 1.0210774019643543;
    lleg_point.positions[4] = -0.50147038258197;
    lleg_point.positions[5] = 0.00083149167042329;
    lleg_goal.trajectory.points.resize(1);
    lleg_goal.trajectory.points[0] = lleg_point;
    lleg_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_leftleg->sendGoal(lleg_goal);
    //Right Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal rleg_goal;
    rleg_goal.goal_time_tolerance = ros::Duration(2.0 * dt);
    rleg_goal.trajectory.header.stamp = ros::Time::now();
    rleg_goal.trajectory.joint_names = {"leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint"};
    trajectory_msgs::JointTrajectoryPoint rleg_point;
    rleg_point.positions.resize(6);
    rleg_point.positions[0] = 7.465561660247033e-05;
    rleg_point.positions[1] = 0.0009297507374013136;
    rleg_point.positions[2] = -0.5196141179915381;
    rleg_point.positions[3] = 1.0210372802461318;
    rleg_point.positions[4] = -0.5012299397574074;
    rleg_point.positions[5] = -0.0009307856459050967;
    rleg_goal.trajectory.points.resize(1);
    rleg_goal.trajectory.points[0] = rleg_point;
    rleg_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    ac_rightleg->sendGoal(rleg_goal);

}

void talos::controlCb(const whole_body_ik_msgs::HumanoidGoalConstPtr &msg)
{
    std::cout<<"Received "<<std::endl;
    //Define Tasks for Whole Body Control
    Eigen::Vector3d vdes;
    int task_type; //0 for linear velocity/ 1 for angular velocity / 2 for CoM Linear Task

    //Left Foot Task
    if( msg->LLeg.linear_velocity.weight >0 && msg->LLeg.linear_velocity.gain > 0)
    {
        vdes = Vector3d(msg->LLeg.linear_velocity.x, msg->LLeg.linear_velocity.y, msg->LLeg.linear_velocity.z);
        task_type = 0; 
        pin->setTask(lfoot_frame, task_type, vdes, msg->LLeg.linear_velocity.weight, msg->LLeg.linear_velocity.gain);
    }

    if(msg->LLeg.angular_velocity.weight > 0 && msg->LLeg.angular_velocity.gain > 0)
    {
        vdes = Vector3d(msg->LLeg.angular_velocity.x, msg->LLeg.angular_velocity.y, msg->LLeg.angular_velocity.z);
        task_type = 1; 
        pin->setTask(lfoot_frame, task_type, vdes, msg->LLeg.angular_velocity.weight, msg->LLeg.angular_velocity.gain);
    }

    //Right Foot Support task
    if( msg->RLeg.linear_velocity.weight >0 && msg->RLeg.linear_velocity.gain > 0)
    {
        task_type = 0; 
        vdes = Vector3d(msg->RLeg.linear_velocity.x, msg->RLeg.linear_velocity.y, msg->RLeg.linear_velocity.z);
        pin->setTask(lfoot_frame, task_type, vdes, msg->RLeg.linear_velocity.weight, msg->RLeg.linear_velocity.gain);
    }
    if(msg->RLeg.angular_velocity.weight > 0 && msg->RLeg.angular_velocity.gain > 0)
    {
        task_type = 1; 
        vdes = Vector3d(msg->RLeg.angular_velocity.x, msg->RLeg.angular_velocity.y, msg->RLeg.angular_velocity.z);
        pin->setTask(lfoot_frame, task_type, vdes, msg->RLeg.angular_velocity.weight, msg->RLeg.angular_velocity.gain);
    }
    //CoM task
    // if(msg->CoM.linear_velocity.weight > 0 &&  msg->CoM.linear_velocity.gain > 0)
    // {
    //     task_type = 2; 
    //     vdes = Vector3d(msg->CoM.linear_velocity.x, msg->CoM.linear_velocity.y, msg->CoM.linear_velocity.z);
    //     pin->setTask("CoM", task_type, vdes,  msg->CoM.linear_velocity.weight,  msg->CoM.linear_velocity.gain);
    // }
    //Torso 
    if(msg->Torso.linear_velocity.weight > 0 && msg->Torso.linear_velocity.gain > 0)
    {
        task_type = 0; 
        vdes =  Vector3d(msg->Torso.linear_velocity.x, msg->Torso.linear_velocity.y, msg->Torso.linear_velocity.z);
        pin->setTask(base_link_frame, task_type, vdes, msg->Torso.linear_velocity.weight, msg->Torso.linear_velocity.gain);
    }
    if(msg->Torso.angular_velocity.weight > 0 && msg->Torso.angular_velocity.gain > 0)
    {
        task_type = 1; 
        vdes =  Vector3d(msg->Torso.angular_velocity.x, msg->Torso.angular_velocity.y, msg->Torso.angular_velocity.z);
        pin->setTask(base_link_frame, task_type, vdes, msg->Torso.angular_velocity.weight, msg->Torso.angular_velocity.gain);
    }
    //Head
    if(msg->Head.linear_velocity.weight > 0 && msg->Head.linear_velocity.gain > 0)
    {
        task_type = 0; 
        vdes = Vector3d(msg->Head.linear_velocity.x, msg->Head.linear_velocity.y, msg->Head.linear_velocity.z);
        pin->setTask(head_frame, task_type, vdes, msg->Head.linear_velocity.weight, msg->Head.linear_velocity.gain);
    }
    if(msg->Head.angular_velocity.weight > 0 && msg->Head.angular_velocity.gain > 0)
    {
        task_type = 1;
        vdes = Vector3d(msg->Head.angular_velocity.x, msg->Head.angular_velocity.y, msg->Head.angular_velocity.z);
        pin->setTask(head_frame, task_type, vdes, msg->Head.angular_velocity.weight, msg->Head.angular_velocity.gain);
    }
    //Right Hand
    if(msg->RHand.linear_velocity.weight>0 &&  msg->RHand.linear_velocity.gain>0)
    {
        task_type = 0; 
        vdes = Vector3d(msg->RHand.linear_velocity.x, msg->RHand.linear_velocity.y, msg->RHand.linear_velocity.z);
        pin->setTask(rhand_frame, task_type, vdes,  msg->RHand.linear_velocity.weight, msg->RHand.linear_velocity.gain);
    }
    if(msg->LHand.angular_velocity.weight>0 && msg->LHand.angular_velocity.gain>0)
    {
        task_type = 1; 
        vdes = Vector3d(msg->RHand.angular_velocity.x, msg->RHand.angular_velocity.y, msg->RHand.angular_velocity.z);
        pin->setTask(rhand_frame, task_type, vdes, msg->RHand.angular_velocity.weight, msg->RHand.angular_velocity.gain);
    }

    //Left Hand
    if(msg->LHand.linear_velocity.weight>0 &&  msg->LHand.linear_velocity.gain>0)
    {
        task_type = 0; 
        vdes = Vector3d(msg->LHand.linear_velocity.x, msg->LHand.linear_velocity.y, msg->LHand.linear_velocity.z);
        pin->setTask(lhand_frame, task_type, vdes,  msg->LHand.linear_velocity.weight, msg->LHand.linear_velocity.gain);
    }
    if(msg->LHand.angular_velocity.weight>0 && msg->LHand.angular_velocity.gain>0)
    {
        task_type = 1; 
        vdes = Vector3d(msg->LHand.angular_velocity.x, msg->LHand.angular_velocity.y, msg->LHand.angular_velocity.z);
        pin->setTask(lhand_frame, task_type, vdes, msg->LHand.angular_velocity.weight, msg->LHand.angular_velocity.gain);
    }

    pin->inverseKinematics(msg->dt);
    pin->printDesiredJointData();

    //Send Desired Joints to Talos
    //Head Joint Controller
    control_msgs::FollowJointTrajectoryGoal head_goal;
    head_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
    head_goal.trajectory.header.stamp = ros::Time::now();
    head_goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
    trajectory_msgs::JointTrajectoryPoint head_point;
    head_point.positions.resize(2);
    head_point.positions[0] = pin->getQd("head_1_joint");
    head_point.positions[1] = pin->getQd("head_2_joint");
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0] = head_point;
    head_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_head->sendGoal(head_goal);
    //Torso Joint Controller
    control_msgs::FollowJointTrajectoryGoal torso_goal;
    torso_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
    torso_goal.trajectory.header.stamp = ros::Time::now();
    torso_goal.trajectory.joint_names = {"torso_1_joint", "torso_2_joint"};
    trajectory_msgs::JointTrajectoryPoint torso_point;
    torso_point.positions.resize(2);
    torso_point.positions[0] = pin->getQd("torso_1_joint");
    torso_point.positions[1] = pin->getQd("torso_2_joint");
    torso_goal.trajectory.points.resize(1);
    torso_goal.trajectory.points[0] = torso_point;
    torso_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_torso->sendGoal(torso_goal);
    //Left Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal larm_goal;
    larm_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
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
    larm_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_leftarm->sendGoal(larm_goal);
    //Right Arm Joint Controller
    control_msgs::FollowJointTrajectoryGoal rarm_goal;
    rarm_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
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
    rarm_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_rightarm->sendGoal(rarm_goal);
    //Left Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal lleg_goal;
    lleg_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
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
    lleg_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_leftleg->sendGoal(lleg_goal);
    //Right Leg Joint Controller
    control_msgs::FollowJointTrajectoryGoal rleg_goal;
    rleg_goal.goal_time_tolerance = ros::Duration(2.0 * msg->dt);
    rleg_goal.trajectory.header.stamp = ros::Time::now();
    rleg_goal.trajectory.joint_names = {"leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint"};
    trajectory_msgs::JointTrajectoryPoint rleg_point;
    rleg_point.positions.resize(6);
    rleg_point.positions[0] = pin->getQd("leg_right_1_joint");
    rleg_point.positions[1] = pin->getQd("leg_right_2_joint");
    rleg_point.positions[2] = pin->getQd("leg_right_3_joint");
    rleg_point.positions[3] = pin->getQd("leg_right_4_joint");
    rleg_point.positions[4] = pin->getQd("leg_right_5_joint");
    rleg_point.positions[5] = pin->getQd("leg_right_6_joint");
    rleg_goal.trajectory.points.resize(1);
    rleg_goal.trajectory.points[0] = rleg_point;
    rleg_goal.trajectory.points[0].time_from_start = ros::Duration(msg->dt);
    ac_rightleg->sendGoal(rleg_goal);
    feedback_.percent_completed = 100;
    as_->publishFeedback(feedback_);
    result_.status = 1;
    as_->setSucceeded(result_);
}




talos::~talos()
{
    delete pin;
}