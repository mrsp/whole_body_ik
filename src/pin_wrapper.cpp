#include <whole_body_ik/pin_wrapper.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <string>
#include <vector>
#include <map>
#include <qpmad/solver.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

using namespace std;

pin_wrapper::pin_wrapper(const std::string &model_name, const bool &has_floating_base_, const bool &verbose)
{
    has_floating_base = has_floating_base_;
    pmodel_ = new pinocchio::Model();

    if (has_floating_base)
        // TODO: Check the Joint for FreeFlyer, if it is correct
        pinocchio::urdf::buildModel(model_name, pinocchio::JointModelFreeFlyer(),
                                    *pmodel_, verbose);
    else
        pinocchio::urdf::buildModel(model_name, *pmodel_, verbose);

    data_ = new pinocchio::Data(*pmodel_);

    jnames_.clear();
    int names_size = pmodel_->names.size();
    jnames_.reserve(names_size);

    for (int i = 0; i < names_size; i++)
    {
        const std::string &jname = pmodel_->names[i];
        std::cout << "Joint Id: " << pmodel_->getJointId(jname) << " # " << pmodel_->idx_qs[pmodel_->getJointId(jname)] << " " << pmodel_->idx_vs[pmodel_->getJointId(jname)] << " Name: " << jname << std::endl;
        //Do not insert "universe" joint
        if (jname.compare("universe") != 0)
        {
            jnames_.push_back(jname);
        }
    }

    qmin_.resize(pmodel_->nq);
    qmax_.resize(pmodel_->nq);
    dqmax_.resize(pmodel_->nv);
    qmin_ = pmodel_->lowerPositionLimit;
    qmax_ = pmodel_->upperPositionLimit;
    dqmax_ = pmodel_->velocityLimit;

    // Continuous joints are given spurious values par default, set those values
    // to arbitrary ones
    // for (int i = 0; i < qmin_.size(); ++i)
    // {
    //     double d = qmax_[i] - qmin_[i];
    //     //If wrong values or if difference less than 0.05 deg (0.001 rad)
    //     if ((d < 0) || (fabs(d) < 0.001))
    //     {
    //         qmin_[i] = -50.0;
    //         qmax_[i] = 50.0;
    //         dqmax_[i] = 200.0;
    //     }
    //     //std::cout << qmin_[i] << "\t" << qmax_[i] << "\t"<< dqmax_[i] << std::endl;
    // }

    I.setIdentity(pmodel_->nv, pmodel_->nv);
    H.setZero(pmodel_->nv, pmodel_->nv);
    h.setZero(pmodel_->nv);
    qd.setZero(pmodel_->nq);
    q_.setZero(pmodel_->nq);
    qdotd.setZero(pmodel_->nv);
    qdot_.setZero(pmodel_->nv);
    jointDataReceived = false;

    lm_damping = 1e-2;
    gainC = 0.5;

    //Not used in QP
    A.resize(0, 0);
    Alb.resize(0);
    Aub.resize(0);
    cost = 1e6;

    //Joint Angle Constraints
    lbq.resize(pmodel_->nv);
    lbq.setZero();
    ubq.resize(pmodel_->nv);
    ubq.setZero();
    //Joint Velocity Constraints
    lbdq.resize(pmodel_->nv);
    lbdq.setZero();
    ubdq.resize(pmodel_->nv);
    ubdq.setZero();

    lbdq = -jointVelocityLimits();
    ubdq = jointVelocityLimits();
    //Composite Joint Constraints
    lb.resize(pmodel_->nv);
    lb = lbdq;
    ub.resize(pmodel_->nv);
    ub = ubdq;
    //solver_params.hessian_type_ = qpmad::SolverParameters::HessianType::HESSIAN_CHOLESKY_FACTOR;
    initialized = false;
    std::cout << "Joint Names " << std::endl;
    printJointNames();
    std::cout << "with " << ndofActuated() << " actuated joints" << std::endl;
    printJointLimits();
    std::cout << "Model loaded: " << model_name << std::endl;
}

void pin_wrapper::updateJointConfig(const std::vector<std::string> &jnames,
                                    const std::vector<double> &qvec,
                                    const std::vector<double> &qdotvec,
                                    double joint_std)
{
    jointDataReceived = true;
    mapJointNamesIDs(jnames, qvec, qdotvec);

    if (has_floating_base)
    {

        //position
        q_[0] = pwb(0);
        q_[1] = pwb(1);
        q_[2] = pwb(2);
        q_[3] = qwb.x(); //x
        q_[4] = qwb.y(); //y
        q_[5] = qwb.z(); //z
        q_[6] = qwb.w(); //w

        //Velocity
        qdot_[0] = vwb(0);
        qdot_[1] = vwb(1);
        qdot_[2] = vwb(2);
        qdot_[3] = omegawb(0); //x
        qdot_[4] = omegawb(1); //y
        qdot_[5] = omegawb(2); //z
    }

    pinocchio::framesForwardKinematics(*pmodel_, *data_, q_);
    pinocchio::computeJointJacobians(*pmodel_, *data_, q_);
}

void pin_wrapper::updateJointConfig(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    q_.setZero(pmodel_->nq);
    qdot_.setZero(pmodel_->nv);
    q_ = q;
    q_[3] = q[4];
    q_[4] = q[5];
    q_[5] = q[6];
    q_[6] = q[3];
    //cout<<"Joint Data Pre"<<q.transpose()<<endl;

    //cout<<"Joint Data After"<<q_.transpose()<<endl;
    qdot_ = dq;

    jointDataReceived = true;
    pinocchio::framesForwardKinematics(*pmodel_, *data_, q_);
    pinocchio::computeJointJacobians(*pmodel_, *data_, q_);


    // std::cout<<" LLeg "<<linkPosition("l_foot").transpose()<<std::endl;
    // std::cout<<" RLeg "<<linkPosition("r_foot").transpose()<<std::endl;
    // std::cout<<" Base "<<linkPosition("pelvis").transpose()<<std::endl;
    // Eigen::Quaterniond qdes = linkOrientation("pelvis");
    // std::cout<<" Base "<< qdes.x()<< qdes.y() << qdes.z() <<qdes.w() << endl;

}

void pin_wrapper::mapJointNamesIDs(const std::vector<std::string> &jnames,
                                   const std::vector<double> &qvec,
                                   const std::vector<double> &qdotvec)
{
    assert(qvec.size() == jnames.size() && qdotvec.size() == jnames.size());

    q_.setZero(pmodel_->nq);
    qdot_.setZero(pmodel_->nv);

    for (int i = 0; i < jnames.size(); i++)
    {
        int jidx = pmodel_->getJointId(jnames[i]);
        int qidx = pmodel_->idx_qs[jidx];
        int vidx = pmodel_->idx_vs[jidx];

        //this value is equal to 2 for continuous joints
        if (pmodel_->nqs[jidx] == 2)
        {
            q_[qidx] = cos(qvec[i]);
            q_[qidx + 1] = sin(qvec[i]);
            qdot_[vidx] = qdotvec[i];
        }
        else
        {
            q_[qidx] = qvec[i];
            qdot_[vidx] = qdotvec[i];
        }
    }
}

void pin_wrapper::getJointData(const std::vector<std::string> &jnames_,
                               std::vector<double> &qvec,
                               std::vector<double> &qdotvec)
{
    qvec.clear();
    qdotvec.clear();
    qvec.resize(jnames_.size());
    qdotvec.resize(jnames_.size());

    for (int i = 0; i < jnames_.size(); i++)
    {
        int jidx = pmodel_->getJointId(jnames_[i]);
        int vidx = pmodel_->idx_vs[jidx];
        int qidx = pmodel_->idx_qs[jidx];

        qvec[i] = q_[qidx];
        qdotvec[i] = qdot_[vidx];
    }
}

void pin_wrapper::getDesiredJointData(Eigen::VectorXd &qvec,
                                      Eigen::VectorXd &qdotvec)
{
    qvec = qd;
    qdotvec = qdotd;
}

void pin_wrapper::getDesiredJointData(const std::vector<std::string> &jnames_,
                                      std::vector<double> &qvec,
                                      std::vector<double> &qdotvec)
{
    qdotvec.clear();
    qvec.clear();
    qdotvec.resize(jnames_.size());
    qvec.resize(jnames_.size());

    for (int i = 0; i < jnames_.size(); i++)
    {
        int jidx = pmodel_->getJointId(jnames_[i]);
        int vidx = pmodel_->idx_vs[jidx];
        int qidx = pmodel_->idx_qs[jidx];
        qdotvec[i] = qdotd(vidx);
        qvec[i] = qd(qidx);
    }
}

double pin_wrapper::getQdotd(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int vidx = pmodel_->idx_vs[jidx];
    return qdotd(vidx);
}

double pin_wrapper::getQd(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int qidx = pmodel_->idx_qs[jidx];
    return qd(qidx);
}

double pin_wrapper::getQdot(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int vidx = pmodel_->idx_vs[jidx];
    return qdot_(vidx);
}

double pin_wrapper::getQ(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int qidx = pmodel_->idx_qs[jidx];
    return q_(qidx);
}

int pin_wrapper::getJointId(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int vidx = pmodel_->idx_vs[jidx];
    return vidx;
}

Eigen::MatrixXd pin_wrapper::geometricJacobian(const std::string &frame_name)
{

    pinocchio::Data::Matrix6x J(6, pmodel_->nv);
    J.fill(0);
    if (has_floating_base)
    {
        try
        {
            // Jacobian in pinocchio::WORLD (link) frame
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::WORLD, J);
            return J;
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(6, ndofActuated());
        }
    }
    else
    {
        try
        {
            // Jacobian in pinocchio::LOCAL (link) frame
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);
            // Transform Jacobians from pinocchio::LOCAL frame to base frame
            J.topRows(3) = (data_->oMf[link_number].rotation()) * J.topRows(3);
            J.bottomRows(3) = (data_->oMf[link_number].rotation()) * J.bottomRows(3);
            return J;
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(6, ndofActuated());
        }
    }
}

Eigen::Vector3d pin_wrapper::linkPosition(const std::string &frame_name)
{

    try
    {
        pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
        return data_->oMf[link_number].translation();
    }
    catch (std::exception &e)
    {
        std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                  << "Returning zeros" << std::endl;
        return Eigen::Vector3d::Zero();
    }
}

Eigen::Quaterniond pin_wrapper::linkOrientation(const std::string &frame_name)
{

    try
    {
        pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
        Eigen::Vector4d temp = rotationToQuaternion(data_->oMf[link_number].rotation());
        Eigen::Quaterniond tempQ;
        tempQ.w() = temp(0);
        tempQ.x() = temp(1);
        tempQ.y() = temp(2);
        tempQ.z() = temp(3);
        return tempQ;
    }
    catch (std::exception &e)
    {
        std::cerr << "WARNING: Frame name " << frame_name << " is invalid! ... "
                  << "Returning zeros" << std::endl;
        return Eigen::Quaterniond::Identity();
    }
}

Eigen::VectorXd pin_wrapper::linkPose(const std::string &frame_name)
{
    Eigen::VectorXd lpose(7);
    lpose.setZero();

    try
    {
        pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
        lpose.head(3) = data_->oMf[link_number].translation();
        lpose.tail(4) = rotationToQuaternion(data_->oMf[link_number].rotation());
    }
    catch (const std::exception &e)
    {
        std::cerr << "WARNING: Frame name " << frame_name << " is invalid! ... "
                  << "Returning zeros" << std::endl;
    }

    return lpose;
}

	/** @brief Computes the logarithmic map for a component in SO(3) group
	 *  @param q Quaternion in SO(3) group
	 *  @return   3D twist in so(3) algebra
	 */
	 Eigen::Vector3d pin_wrapper::logMap(
			Eigen::Quaterniond q) {

		Eigen::Vector3d omega;
		omega = Eigen::Vector3d::Zero();

		double temp = q.norm();

		Eigen::Vector3d tempV = Eigen::Vector3d(q.x(), q.y(), q.z());

		double temp_ = tempV.norm();
		if (temp_ > std::numeric_limits<double>::epsilon()) 
		{
			tempV *= (1.000 / temp_);
			omega = tempV * (2.0 * acos(q.w() / temp));
		}


		return omega;
	}
Eigen::MatrixXd pin_wrapper::linearJacobian(const std::string &frame_name)
{

    pinocchio::Data::Matrix6x J(6, ndofActuated());
    J.fill(0);

    if (has_floating_base)
    {
        try
        {
            // Jacobian in pinocchio::WORLD frame
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::WORLD, J);
            return J.topRows(3);
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(3, ndofActuated());
        }
    }
    else
    {
        try
        {
            // Jacobian in pinocchio::LOCAL frame
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);
            // Transform Jacobian from pinocchio::LOCAL frame to base frame
            return (data_->oMf[link_number].rotation()) * J.topRows(3);
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(3, ndofActuated());
        }
    }
}

Eigen::MatrixXd pin_wrapper::angularJacobian(const std::string &frame_name)
{
    pinocchio::Data::Matrix6x J(6, pmodel_->nv);
    J.fill(0);
    if (has_floating_base)
    {
        try
        {
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            // Jacobian in pinocchio::WORLD frame
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::WORLD, J);
            return J.bottomRows(3);
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(3, ndofActuated());
        }
    }
    else
    {
        try
        {
            pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
            // Jacobian in pinocchio::LOCAL frame
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);
            // Transform Jacobian from pinocchio::LOCAL frame to base frame
            return (data_->oMf[link_number].rotation()) * J.bottomRows(3);
        }
        catch (std::exception &e)
        {
            std::cerr << "WARNING: Link name " << frame_name << " is invalid! ... "
                      << "Returning zeros" << std::endl;
            return Eigen::MatrixXd::Zero(3, ndofActuated());
        }
    }
}

Eigen::VectorXd pin_wrapper::comPosition()
{
    Eigen::Vector3d com;
    pinocchio::centerOfMass(*pmodel_, *data_, q_);
    com = data_->com[0];
    return com;
}

Eigen::MatrixXd pin_wrapper::comJacobian() const
{
    Eigen::MatrixXd Jcom;
    Jcom.setZero(3, pmodel_->nv);
    pinocchio::jacobianCenterOfMass(*pmodel_, *data_, q_);
    Jcom = data_->Jcom;
    return Jcom;
}

void pin_wrapper::printJointLimits() const
{
    if (!((jnames_.size() == qmin_.size()) &&
          (jnames_.size() == qmax_.size()) &&
          (jnames_.size() == dqmax_.size())))
    {
        std::cerr << "Joint names and joint limits size do not match!"
                  << std::endl;
        return;
    }
    std::cout << "\nJoint Name\t qmin \t qmax \t dqmax" << std::endl;
    for (int i = 0; i < jnames_.size(); ++i)
        std::cout << jnames_[i] << "\t\t" << qmin_[i] << "\t" << qmax_[i] << "\t"
                  << dqmax_[i] << std::endl;
}

void pin_wrapper::printDesiredJointData() const
{
    std::cout << "Joint name"
              << "\t\t\t"
              << "D. Joint Position"
              << "\t\t\t\t"
              << "D. Joint Velocity" << std::endl;

    for (int i = 0; i < jnames_.size(); ++i)
        std::cout << jnames_[i] << "\t\t\t" << getQd(jnames_[i]) << "\t\t\t\t" << getQdotd(jnames_[i]) << std::endl;
}

void pin_wrapper::printActualJointData() const
{
    std::cout << "Joint name"
              << "\t\t\t"
              << "A. Joint Position"
              << "\t\t\t\t"
              << "A. Joint Velocity" << std::endl;

    for (int i = 0; i < jnames_.size(); ++i)
        std::cout << jnames_[i] << "\t\t\t" << getQ(jnames_[i]) << "\t\t\t\t" << getQdot(jnames_[i]) << std::endl;
}

Eigen::Vector4d pin_wrapper::rotationToQuaternion(const Eigen::Matrix3d &R)
{
    double dEpsilon = 1e-6;
    Eigen::Vector4d quat;

    quat(0) = 0.5 * sqrt(R(0, 0) + R(1, 1) + R(2, 2) + 1.0);
    if (fabs(R(0, 0) - R(1, 1) - R(2, 2) + 1.0) < dEpsilon)
        quat(1) = 0.0;
    else
        quat(1) = 0.5 * sgn(R(2, 1) - R(1, 2)) * sqrt(R(0, 0) - R(1, 1) - R(2, 2) + 1.0);
    if (fabs(R(1, 1) - R(2, 2) - R(0, 0) + 1.0) < dEpsilon)
        quat(2) = 0.0;
    else
        quat(2) = 0.5 * sgn(R(0, 2) - R(2, 0)) * sqrt(R(1, 1) - R(2, 2) - R(0, 0) + 1.0);
    if (fabs(R(2, 2) - R(0, 0) - R(1, 1) + 1.0) < dEpsilon)
        quat(3) = 0.0;
    else
        quat(3) = 0.5 * sgn(R(1, 0) - R(0, 1)) * sqrt(R(2, 2) - R(0, 0) - R(1, 1) + 1.0);

    return quat;
}

Eigen::Matrix3d pin_wrapper::quaternionToRotation(const Eigen::Vector4d &q)
{
    double normq = q.norm();
    if (fabs(normq - 1.0) > 0.001)
    {
        std::cerr << "WARNING: Input quaternion is not unitary! ... "
                  << "Returning identity" << std::endl;
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix3d res;
    res(0, 0) = 2.0 * (q(0) * q(0) + q(1) * q(1)) - 1.0;
    res(0, 1) = 2.0 * (q(1) * q(2) - q(0) * q(3));
    res(0, 2) = 2.0 * (q(1) * q(3) + q(0) * q(2));
    res(1, 0) = 2.0 * (q(1) * q(2) + q(0) * q(3));
    res(1, 1) = 2.0 * (q(0) * q(0) + q(2) * q(2)) - 1.0;
    res(1, 2) = 2.0 * (q(2) * q(3) - q(0) * q(1));
    res(2, 0) = 2.0 * (q(1) * q(3) - q(0) * q(2));
    res(2, 1) = 2.0 * (q(2) * q(3) + q(0) * q(1));
    res(2, 2) = 2.0 * (q(0) * q(0) + q(3) * q(3)) - 1.0;

    return res;
}

void pin_wrapper::clearTasks()
{
    cost_ = cost;
    cost = 0;
    H.setZero(pmodel_->nv, pmodel_->nv);
    h.setZero(pmodel_->nv);
}

void pin_wrapper::setLinearTask(const std::string &frame_name, int task_type, Eigen::Vector3d vdes, Eigen::Vector3d pdes, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM" << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    Eigen::Vector3d e, pmeas, vmeas;
    Jac.resize(3, pmodel_->nv);

    if (task_type == 0)
    {
        Jac = linearJacobian(frame_name);
        pmeas = linkPosition(frame_name);
    }
    else if (task_type == 2)
    {
        Jac = comJacobian();
        pmeas = comPosition();
    }
    e = (pdes - pmeas);
    vdes = gain * e/dt;

    //Measured Link's Linear Velocity in the world frame;
 
        vmeas = Jac * qdot_;
        // cout<<"Frame \n"<<frame_name<<endl;
        // cout<<"des \n"<<pdes.transpose()<<endl;
        // cout<<"actual \n"<<pmeas.transpose()<<endl;
        // cout<<"JACOBIAN \n"<<Jac<<endl;
  
    cost += (vmeas - vdes).squaredNorm() * weight;
    H += weight * Jac.transpose() * Jac + lm_damping * fmax(lm_damping, e.norm()) * I; //fmax(lm_damping, v.norm())
    h -= (weight * vdes.transpose() * Jac).transpose();
}

void pin_wrapper::setAngularTask(const std::string &frame_name, int task_type, Eigen::Vector3d wdes, Eigen::Quaterniond qdes, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM" << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    Eigen::Vector3d e, wmeas, v_d, v_;
    Jac.resize(3, pmodel_->nv);

    //Jacobian in World Frame
    Jac = angularJacobian(frame_name);

    //Orientation in World Frame
    Eigen::Quaterniond qmeas = linkOrientation(frame_name);


    e = logMap(qdes * qmeas.inverse());
    // v_d = Eigen::Vector3d(qdes.x(), qdes.y(), qdes.z());
    // v_ = Eigen::Vector3d(qmeas.x(), qmeas.y(), qmeas.z());
    // e = -(qdes.w() * v_ - qmeas.w() * v_d + wedge(v_d) * v_) / dt;
    // cout<<"Frame \n"<<frame_name<<endl;
    // cout<<"des \n"<< qdes.x()<< qdes.y() << qdes.z() <<qdes.w() << endl;
    // cout<<"actual \n"<< qmeas.x()<< qmeas.y() << qmeas.z() <<qmeas.w() << endl;
    // cout<<"error \n" <<e.transpose()<<endl;
    //Error in World Frame
    wdes = gain * e/dt;
    wmeas = Jac * qdot_;

    cost += (wdes - wmeas).squaredNorm() * weight;
    H += weight * Jac.transpose() * Jac + lm_damping * fmax(lm_damping, e.norm()) * I; //fmax(lm_damping, v.norm())
    h -= (weight * wdes.transpose() * Jac).transpose();
}
void pin_wrapper::setDOFTask(const std::string &joint_name, int task_type, double qqdes, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM / 3 for DOF " << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    double v, qq, qqdot;
    Jac.resize(1, pmodel_->nv);
    Jac.setZero();

    int idx = getJointId(joint_name);

    Jac(0, idx) = 1;
    qq = getQ(joint_name);
    v = (qqdes - qq) / dt;

    qqdot = getQdot(joint_name);
    double cost__ = (qqdot - gain * v) * (qqdot - gain * v) * weight;
    cost += cost__;

    //std::cout<<"Joint "<<joint_name<<"Id "<<idx<<" Data "<<qq<<" "<<qqdot<<" Des "<<qqdes<<" Cost "<<cost__<<std::endl;

    H += weight * Jac.transpose() * Jac + lm_damping * fmax(lm_damping, fabs(v)) * I;
    h -= (weight * gain * v * Jac).transpose();
}

void pin_wrapper::addTasks(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask, double dt)
{
    unsigned int i = 0;
    while (i < ltask.size())
    {
        setLinearTask(ltask[i].frame_name, ltask[i].task_type, ltask[i].vdes, ltask[i].des, ltask[i].weight, ltask[i].gain, dt);
        i++;
    }
    i = 0;
    while (i < atask.size())
    {
        setAngularTask(atask[i].frame_name, atask[i].task_type, atask[i].wdes, atask[i].qdes, atask[i].weight, atask[i].gain, dt);
        i++;
    }
    i = 0;
    while (i < dtask.size())
    {
        setDOFTask(dtask[i].joint_name, dtask[i].task_type, dtask[i].des, dtask[i].weight, dtask[i].gain, dt);
        i++;
    }
}
void pin_wrapper::addReguralization()
{
    H += I * 2.2e-6;
}

void pin_wrapper::setBaseToWorldTransform(Eigen::Affine3d Twb_)
{
    pwb = Twb_.translation();
    Rwb = Twb_.linear();
    qwb = Eigen::Quaterniond(Rwb);
    Twb = Twb_;
}
void pin_wrapper::setBaseWorldVelocity(Eigen::Vector3d vwb_, Eigen::Vector3d omegawb_)
{
    vwb = vwb_;
    omegawb = omegawb_;
}

Eigen::VectorXd pin_wrapper::inverseKinematics(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask, double dt)
{

    unsigned int j = 0;
    if (!jointDataReceived)
    {
        std::cout << "Joint Data not received " << std::endl;
        return qdot_;
    }

  
    jointDataReceived = false;

    if(!initialized)
        qd = q_;


    int iter = 0;
    int ii=0;
    if(has_floating_base)
       ii = 6;

    // while (iter < 1000)
    // {
        clearTasks();
        addTasks(ltask, atask, dtask, dt);
        //addReguralization();
        lbq = gainC * (jointMinAngularLimits() - q_) / dt;
        ubq = gainC * (jointMaxAngularLimits() - q_) / dt;


        for (unsigned int i = ii; i < pmodel_->nv; ++i)
        {
            lb(i) = lbq(i) > lbdq(i) ? lbq(i) : lbdq(i);
            ub(i) = ubq(i) < ubdq(i) ? ubq(i) : ubdq(i);
        }
     
        // cout<<"H "<<H<<endl;
        // cout<<"h "<<h<<endl;
        //qdotd = H.colPivHouseholderQr().solve(-h);
        //qdotd = H.inverse()*h;
        // std::cout << "Unconstrained Optimal Solution" << qdotd << std::endl;
        //cholesky.compute(H);
        //qdotd_ = cholesky.solve(-h);
        //std::cout << "Unconstrained Optimal Solution" << qdotd << std::endl;
        //std::cout << "-----" << std::endl;
        //L_choleksy = Eigen::MatrixXd(cholesky.matrixL());
        //qpmad::Solver::ReturnStatus status = solver.solve(qdotd, L_choleksy, h, lb, ub, A, Alb, Aub, solver_params);

        qpmad::Solver::ReturnStatus status = solver.solve(qdotd, H, h, lb, ub);

        qd = pinocchio::integrate(*pmodel_, q_, qdotd * dt);


        // for (int i = 0; i < qd.size(); ++i)
        //     std::cout << qd[i] << std::endl;
        // std::cout<<"Optmization Velocity  ---"<<std::endl;
        // for (int i = 0; i < qdotd.size(); ++i)
        //     std::cout << qdotd[i] << std::endl;

    //     double impr = fabs(cost - cost_) / cost_;
    //     std::cout << "Cost " << cost << " Improv " << impr << std::endl;

    //     if (impr < 1.0e-5 || fabs(cost) < 1.0e-6)
    //     {
    //         break;
    //     }

    //     pinocchio::framesForwardKinematics(*pmodel_, *data_, qd);
    //     pinocchio::computeJointJacobians(*pmodel_, *data_, qd);
    //     q_ = qd;
    //     qdot_ = qdotd;
    //     iter++;
    // }
    //std::cout << "---------------" << std::endl;

    return qdotd;
}
