#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <whole_body_ik/pin_wrapper.h>
#include <lipm_control/ZMPDistributor.h>
#include <lipm_control/postureStabilizer.h>
#include <lipm_control/LIPMControl.h>
#include <chrono>
#include <whole_body_ik/nao_wbc.h>

using namespace std::chrono;


nao_wbc::nao_wbc(ros::NodeHandle nh_)
{

  nh = nh_;
  ros::NodeHandle n_p("~");
  n_p.param<std::string>("modelname", modelname, "/home/master/catkin_ws/src/whole_body_ik/share/urdf/nao.urdf");
  n_p.param<std::string>("base_link_frame", base_link_frame, "base_link");
  n_p.param<std::string>("lfoot_frame", lfoot_frame, "l_sole");
  n_p.param<std::string>("rfoot_frame", rfoot_frame, "r_sole");
  n_p.param<std::string>("lhand_frame", lhand_frame, "LHand");
  n_p.param<std::string>("rhand_frame", rhand_frame, "RHand");
  n_p.param<std::string>("head_frame", head_frame, "Head");
  n_p.param<double>("control_frequency", joint_freq, 100.0);
  n_p.param<std::string>("desired_joint_state_topic", joint_cmd_topic, "/nao_raisim_ros/command_joint_states");

  pin = new pin_wrapper(modelname, true);
  cmd_pub = nh.advertise<sensor_msgs::JointState>(joint_cmd_topic, 1000);
  as_ = new actionlib::SimpleActionServer<whole_body_ik_msgs::HumanoidAction>(nh, "/nao_raisim_ros/whole_body_control", boost::bind(&nao_wbc::controlCb, this, _1), false);
  as_->start();
  q.resize(26);
  dq.resize(26);


}





void nao_wbc::run()
{
    static ros::Rate rate(joint_freq);
    while (ros::ok())
    {

        rate.sleep();
        ros::spinOnce();
    }
}
void nao_wbc::controlCb(const whole_body_ik_msgs::HumanoidGoalConstPtr &msg)
{


        linearTask ltask;
        angularTask atask;
        dofTask dtask;
        std::vector<linearTask> ltaskVec;
        std::vector<angularTask> ataskVec;
        std::vector<dofTask> dtaskVec;

        pwb = Vector3d(msg->odom.pose.pose.position.x, msg->odom.pose.pose.position.y,msg->odom.pose.pose.position.z);
        vwb = Vector3d(msg->odom.twist.twist.linear.x, msg->odom.twist.twist.linear.y, msg->odom.twist.twist.linear.z);
        omegawb = Vector3d(msg->odom.twist.twist.angular.x, msg->odom.twist.twist.angular.y, msg->odom.twist.twist.angular.z);
        qwb = Eigen::Quaterniond(msg->odom.pose.pose.orientation.w, msg->odom.pose.pose.orientation.x, msg->odom.pose.pose.orientation.y, msg->odom.pose.pose.orientation.z);

        pin->setBaseToWorldState(pwb, qwb);
        pin->setBaseWorldVelocity(vwb, omegawb);

        std::vector<double> pos_vector = msg->joint_state.position;
        double* pos_array = pos_vector.data();
        q = Eigen::Map<Eigen::Matrix<double, 26, 1> >(pos_array);
        std::vector<double> vel_vector = msg->joint_state.velocity;
        double* vel_array = vel_vector.data();
        dq = Eigen::Map<Eigen::Matrix<double, 26, 1> >(vel_array);

        pin->updateJointConfig(msg->joint_state.name, q, dq);



        //Define Tasks for Whole Body Control
        unsigned int j = 0;
        while (j < msg->Joints.size())
        {
            dtask.joint_name = msg->Joints[j].name;
            dtask.des = msg->Joints[j].desired_angle;
            dtask.weight = msg->Joints[j].weight;
            dtask.gain = msg->Joints[j].gain;
            dtask.task_type = 3;
            dtaskVec.push_back(dtask);
            j++;
        }

        //Left Foot Task
        if (msg->LLeg.linear_task.weight > 0 && msg->LLeg.linear_task.gain > 0)
        {
            ltask.frame_name = lfoot_frame;
            ltask.des = Eigen::Vector3d(msg->LLeg.linear_task.desired_position.x, msg->LLeg.linear_task.desired_position.y, msg->LLeg.linear_task.desired_position.z);
            ltask.vdes = Eigen::Vector3d(msg->LLeg.linear_task.desired_linear_velocity.x, msg->LLeg.linear_task.desired_linear_velocity.y, msg->LLeg.linear_task.desired_linear_velocity.z);
            ltask.weight = msg->LLeg.linear_task.weight;
            ltask.gain = msg->LLeg.linear_task.gain;
            ltask.task_type = 0;
            ltaskVec.push_back(ltask);
        }

        if (msg->LLeg.angular_task.weight > 0 && msg->LLeg.angular_task.gain > 0)
        {
            atask.frame_name = lfoot_frame;
            atask.qdes = Eigen::Quaterniond(msg->LLeg.angular_task.desired_orientation.w, msg->LLeg.angular_task.desired_orientation.x, msg->LLeg.angular_task.desired_orientation.y, msg->LLeg.angular_task.desired_orientation.z);
            atask.wdes = Eigen::Vector3d(msg->LLeg.angular_task.desired_angular_velocity.x, msg->LLeg.angular_task.desired_angular_velocity.y, msg->LLeg.angular_task.desired_angular_velocity.z);
            atask.weight = msg->LLeg.angular_task.weight;
            atask.gain = msg->LLeg.angular_task.gain;
            atask.task_type = 1;
            ataskVec.push_back(atask);
        }

        //Right Foot Support task
        if (msg->RLeg.linear_task.weight > 0 && msg->RLeg.linear_task.gain > 0)
        {
            ltask.frame_name = rfoot_frame;
            ltask.des = Eigen::Vector3d(msg->RLeg.linear_task.desired_position.x, msg->RLeg.linear_task.desired_position.y, msg->RLeg.linear_task.desired_position.z);
            ltask.vdes = Eigen::Vector3d(msg->RLeg.linear_task.desired_linear_velocity.x, msg->RLeg.linear_task.desired_linear_velocity.y, msg->RLeg.linear_task.desired_linear_velocity.z);
            ltask.weight = msg->RLeg.linear_task.weight;
            ltask.gain = msg->RLeg.linear_task.gain;
            ltask.task_type = 0;
            ltaskVec.push_back(ltask);
        }
        if (msg->RLeg.angular_task.weight > 0 && msg->RLeg.angular_task.gain > 0)
        {
            atask.frame_name = rfoot_frame;
            atask.qdes = Eigen::Quaterniond(msg->RLeg.angular_task.desired_orientation.w, msg->RLeg.angular_task.desired_orientation.x, msg->RLeg.angular_task.desired_orientation.y, msg->RLeg.angular_task.desired_orientation.z);
            atask.wdes = Eigen::Vector3d(msg->RLeg.angular_task.desired_angular_velocity.x, msg->RLeg.angular_task.desired_angular_velocity.y, msg->RLeg.angular_task.desired_angular_velocity.z);
            atask.weight = msg->RLeg.angular_task.weight;
            atask.gain = msg->RLeg.angular_task.gain;
            atask.task_type = 1;
            ataskVec.push_back(atask);
        }

        //CoM task
        if (msg->CoM.linear_task.weight > 0 && msg->CoM.linear_task.gain > 0)
        {
            ltask.frame_name = "CoM";
            ltask.des = Eigen::Vector3d(msg->CoM.linear_task.desired_position.x, msg->CoM.linear_task.desired_position.y, msg->CoM.linear_task.desired_position.z);
            ltask.vdes = Eigen::Vector3d(msg->CoM.linear_task.desired_linear_velocity.x, msg->CoM.linear_task.desired_linear_velocity.y, msg->CoM.linear_task.desired_linear_velocity.z);
            
            ltask.weight = msg->CoM.linear_task.weight;
            ltask.gain = msg->CoM.linear_task.gain;
            ltask.task_type = 2;
            ltaskVec.push_back(ltask);
        }
        // //Torso
        if (msg->Torso.angular_task.weight > 0 && msg->Torso.angular_task.gain > 0)
        {
            atask.frame_name = base_link_frame;
            atask.qdes = Eigen::Quaterniond(msg->Torso.angular_task.desired_orientation.w, msg->Torso.angular_task.desired_orientation.x, msg->Torso.angular_task.desired_orientation.y, msg->Torso.angular_task.desired_orientation.z);
            atask.wdes = Eigen::Vector3d(msg->Torso.angular_task.desired_angular_velocity.x, msg->Torso.angular_task.desired_angular_velocity.y, msg->Torso.angular_task.desired_angular_velocity.z);
            atask.weight = msg->Torso.angular_task.weight;
            atask.gain = msg->Torso.angular_task.gain;
            atask.task_type = 1;
            ataskVec.push_back(atask);
        }
     
        // //Head
        // if (msg->Head.linear_task.weight > 0 && msg->Head.linear_task.gain > 0)
        // {
        //     ltask.frame_name = head_frame;
        //     ltask.des = Eigen::Vector3d(msg->Head.linear_task.desired_position.x, msg->Head.linear_task.desired_position.y, msg->Head.linear_task.desired_position.z);
        //     ltask.vdes = Eigen::Vector3d(msg->Head.linear_task.desired_linear_velocity.x, msg->Head.linear_task.desired_linear_velocity.y, msg->Head.linear_task.desired_linear_velocity.z);
        //     ltask.weight = msg->Head.linear_task.weight;
        //     ltask.gain = msg->Head.linear_task.gain;
        //     ltask.task_type = 0;
        //     ltaskVec.push_back(ltask);
        // }
        // if (msg->Head.angular_task.weight > 0 && msg->Head.angular_task.gain > 0)
        // {
        //     atask.frame_name = head_frame;
        //     atask.qdes = Eigen::Quaterniond(msg->Head.angular_task.desired_orientation.w, msg->Head.angular_task.desired_orientation.x, msg->Head.angular_task.desired_orientation.y, msg->Head.angular_task.desired_orientation.z);
        //     atask.wdes = Eigen::Vector3d(msg->Head.angular_task.desired_angular_velocity.x, msg->Head.angular_task.desired_angular_velocity.y, msg->Head.angular_task.desired_angular_velocity.z);
        //     atask.weight = msg->Head.angular_task.weight;
        //     atask.gain = msg->Head.angular_task.gain;
        //     atask.task_type = 1;
        //     ataskVec.push_back(atask);
        // }
        // //Left Hand Task
        // if (msg->LHand.linear_task.weight > 0 && msg->LHand.linear_task.gain > 0)
        // {
        //     ltask.frame_name = lhand_frame;
        //     ltask.des = Eigen::Vector3d(msg->LHand.linear_task.desired_position.x, msg->LHand.linear_task.desired_position.y, msg->LHand.linear_task.desired_position.z);
        //     ltask.vdes = Eigen::Vector3d(msg->LHand.linear_task.desired_linear_velocity.x, msg->LHand.linear_task.desired_linear_velocity.y, msg->LHand.linear_task.desired_linear_velocity.z);
        //     ltask.weight = msg->LHand.linear_task.weight;
        //     ltask.gain = msg->LHand.linear_task.gain;
        //     ltask.task_type = 0;
        //     ltaskVec.push_back(ltask);
        // }

        // if (msg->LHand.angular_task.weight > 0 && msg->LHand.angular_task.gain > 0)
        // {
        //     atask.frame_name = lhand_frame;
        //     atask.qdes = Eigen::Quaterniond(msg->LHand.angular_task.desired_orientation.w, msg->LHand.angular_task.desired_orientation.x, msg->LHand.angular_task.desired_orientation.y, msg->LHand.angular_task.desired_orientation.z);
        //     atask.wdes = Eigen::Vector3d(msg->LHand.angular_task.desired_angular_velocity.x, msg->LHand.angular_task.desired_angular_velocity.y, msg->LHand.angular_task.desired_angular_velocity.z);
        //     atask.weight = msg->LHand.angular_task.weight;
        //     atask.gain = msg->LHand.angular_task.gain;
        //     atask.task_type = 1;
        //     ataskVec.push_back(atask);
        // }

        // //Right Hand  task
        // if (msg->RHand.linear_task.weight > 0 && msg->RHand.linear_task.gain > 0)
        // {
        //     ltask.frame_name = rhand_frame;
        //     ltask.des = Eigen::Vector3d(msg->RHand.linear_task.desired_position.x, msg->RHand.linear_task.desired_position.y, msg->RHand.linear_task.desired_position.z);
        //     ltask.vdes = Eigen::Vector3d(msg->RHand.linear_task.desired_linear_velocity.x, msg->RHand.linear_task.desired_linear_velocity.y, msg->RHand.linear_task.desired_linear_velocity.z);
        //     ltask.weight = msg->RHand.linear_task.weight;
        //     ltask.gain = msg->RHand.linear_task.gain;
        //     ltask.task_type = 0;
        //     ltaskVec.push_back(ltask);
        // }
        // if (msg->RHand.angular_task.weight > 0 && msg->RHand.angular_task.gain > 0)
        // {
        //     atask.frame_name = rhand_frame;
        //     atask.qdes = Eigen::Quaterniond(msg->RHand.angular_task.desired_orientation.w, msg->RHand.angular_task.desired_orientation.x, msg->RHand.angular_task.desired_orientation.y, msg->RHand.angular_task.desired_orientation.z);
        //     atask.wdes = Eigen::Vector3d(msg->RHand.angular_task.desired_angular_velocity.x, msg->RHand.angular_task.desired_angular_velocity.y, msg->RHand.angular_task.desired_angular_velocity.z);
        //     atask.weight = msg->RHand.angular_task.weight;
        //     atask.gain = msg->RHand.angular_task.gain;
        //     atask.task_type = 1;
        //     ataskVec.push_back(atask);
        // }



      pin->inverseKinematics(ltaskVec, ataskVec, dtaskVec, msg->dt);
      pin->getDesiredJointData(msg->joint_state.name, qd, dqd);
      swapQuatWXYZ(qd);


    sensor_msgs::JointState joint_msg;
    // from Eigen:vectorXd to pub msg
    std::vector<double> qfoo;//(qd.data()+7, qd.data() + qd.size()-7); 
    qfoo.resize(qd.size()-7);
    for(unsigned int ii=0;ii<qd.size()-7;ii++)
        qfoo[ii] = qd(ii+7);


    joint_msg.position.resize(qfoo.size()); 
    joint_msg.position = qfoo ;

    std::vector<double> dqfoo;//(dqd.data(), dqd.data()  + dqd.size() ); 
    dqfoo.resize(dqd.size()-6);
    for(unsigned int ii=0;ii<dqd.size()-6;ii++)
        dqfoo[ii] = dqd(ii+6);

    joint_msg.velocity.resize(dqfoo.size()); 
    joint_msg.velocity= dqfoo ;

    joint_msg.name.resize(msg->joint_state.name.size()) ;
    joint_msg.name = msg->joint_state.name;
    joint_msg.header.stamp = ros::Time::now();




    cmd_pub.publish(joint_msg);
    feedback_.percent_completed = 100;
    as_->publishFeedback(feedback_);
    result_.status = 1;
    result_.joint_cmd_state = joint_msg;
    as_->setSucceeded(result_);

}

nao_wbc::~nao_wbc()
{

}