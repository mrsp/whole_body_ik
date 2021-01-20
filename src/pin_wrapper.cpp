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

pin_wrapper::pin_wrapper(const std::string &model_name, const bool &has_floating_base, const bool &verbose)
{
    has_floating_base_ = has_floating_base;
    pmodel_ = new pinocchio::Model();

    pinocchio::urdf::buildModel(model_name, *pmodel_, verbose);
    data_ = new pinocchio::Data(*pmodel_);

    jnames_.clear();
    int names_size = pmodel_->names.size();
    jnames_.reserve(names_size);

    for (int i = 0; i < names_size; i++)
    {
        const std::string &jname = pmodel_->names[i];
        //Do not insert "universe" joint
        if (jname.compare("universe") != 0)
        {
            jnames_.push_back(jname);
        }
    }

    qmin_.resize(jnames_.size());
    qmax_.resize(jnames_.size());
    dqmax_.resize(jnames_.size());
    qmin_ = pmodel_->lowerPositionLimit;
    qmax_ = pmodel_->upperPositionLimit;
    dqmax_ = pmodel_->velocityLimit;

    // Continuous joints are given spurious values par default, set those values
    // to arbitrary ones
    for (int i = 0; i < qmin_.size(); ++i)
    {
        double d = qmax_[i] - qmin_[i];
        // If wrong values or if difference less than 0.05 deg (0.001 rad)
        if ((d < 0) || (fabs(d) < 0.001))
        {
            qmin_[i] = -50.0;
            qmax_[i] = 50.0;
            dqmax_[i] = 200.0;
        }
    }

    I.setIdentity(pmodel_->nv, pmodel_->nv);
    H.setZero(pmodel_->nv, pmodel_->nv);
    h.setZero(pmodel_->nv);
    qdotd.setZero(pmodel_->nv);
    qd.setZero(pmodel_->nv);
    q_.setZero(pmodel_->nv);
    iters = 1000;
    lm_damping = 1e-3;
    gainC = 0.65;
    //Not used in QP
    A.resize(0, 0);
    Alb.resize(0);
    Aub.resize(0);
    cost = 0;
    cost_ = 0;
    //Joint Angle Constraints
    lbq.resize(pmodel_->nv);
    ubq.resize(pmodel_->nv);
    //Joint Velocity Constraints
    lbdq.resize(pmodel_->nv);
    ubdq.resize(pmodel_->nv);
    lbdq = -jointVelocityLimits();
    ubdq = jointVelocityLimits();
    //Composite Joint Constraints
    lb.resize(pmodel_->nv);
    ub.resize(pmodel_->nv);

    //solver_params.hessian_type_ = qpmad::SolverParameters::HessianType::HESSIAN_CHOLESKY_FACTOR;

    std::cout << "Joint Names " << std::endl;
    printJointNames();
    std::cout << "with " << ndofActuated() << " actuated joints" << std::endl;
    std::cout << "Model loaded: " << model_name << std::endl;
}

void pin_wrapper::updateJointConfig(const std::vector<std::string> &jnames,
                                    const std::vector<double> &qvec,
                                    const std::vector<double> &qdotvec,
                                    double joint_std)
{
    mapJointNamesIDs(jnames, qvec, qdotvec);
    pinocchio::framesForwardKinematics(*pmodel_, *data_, qd);
    pinocchio::computeJointJacobians(*pmodel_, *data_, qd);

}

void pin_wrapper::forwardKinematics(Eigen::VectorXd pin_joint_pos, Eigen::VectorXd pin_joint_vel)
{
    pinocchio::framesForwardKinematics(*pmodel_, *data_, pin_joint_pos);
    pinocchio::computeJointJacobians(*pmodel_, *data_, pin_joint_pos);
}

void pin_wrapper::mapJointNamesIDs(const std::vector<std::string> &jnames,
                                   const std::vector<double> &qvec,
                                   const std::vector<double> &qdotvec)
{
    assert(qvec.size() == jnames.size() && qdotvec.size() == jnames.size());

    // q_.setZero(pmodel_->nv);
    // qd.setZero(pmodel_->nv);
    // qdotd.setZero(pmodel_->nv);

    for (int i = 0; i < jnames.size(); i++)
    {
        int jidx = pmodel_->getJointId(jnames[i]);
        int qidx = pmodel_->idx_qs[jidx];
        int vidx = pmodel_->idx_vs[jidx];

        //this value is equal to 2 for continuous joints
        if (pmodel_->nqs[jidx] == 2)
        {
            qd[qidx] = cos(qvec[i]);
            qd[qidx + 1] = sin(qvec[i]);
            qdotd[vidx] = qdotvec[i];
        }
        else
        {
            qd[vidx] = qvec[i];
            qdotd[vidx] = qdotvec[i];
        }
    }
    q_ = qd;
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

        qvec[i] = qd[vidx];
        qdotvec[i] = qdotd[vidx];
    }
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

        qdotvec[i] = qdotd(vidx);
        qvec[i] = qd(vidx);
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
    int vidx = pmodel_->idx_vs[jidx];

    return qd(vidx);
}

int pin_wrapper::getJointId(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int vidx = pmodel_->idx_vs[jidx];

    return vidx;
}

Eigen::MatrixXd pin_wrapper::geometricJacobian(const std::string &frame_name)
{
    try
    {
        pinocchio::Data::Matrix6x J(6, pmodel_->nv);
        J.fill(0);
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
    pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);

    lpose.head(3) = data_->oMf[link_number].translation();
    lpose.tail(4) = rotationToQuaternion(data_->oMf[link_number].rotation());

    return lpose;
}

Eigen::Vector3d pin_wrapper::logMap(Eigen::Quaterniond quat)
{

    Eigen::Vector3d omega;
    omega = Eigen::Vector3d::Zero();

    double temp = quat.norm();

    Eigen::Vector3d tempV = Eigen::Vector3d(quat.x(), quat.y(), quat.z());

    double temp_ = tempV.norm();
    tempV *= (1.000 / temp_);

    omega = tempV * (2.0 * acos(quat.w() / temp));
    //omega = tempV * (2.0 * atan2(temp_,q.w()));
    if (std::isnan(omega(0) + omega(1) + omega(2)))
        omega = Eigen::Vector3d::Zero();

    return omega;
}

Eigen::MatrixXd pin_wrapper::linearJacobian(const std::string &frame_name)
{
    pinocchio::Data::Matrix6x J(6, pmodel_->nv);
    J.fill(0);
    pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);

    pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);
    try
    {
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

Eigen::MatrixXd pin_wrapper::angularJacobian(const std::string &frame_name)
{
    pinocchio::Data::Matrix6x J(6, pmodel_->nv);
    J.fill(0);
    pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);
    try
    {
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

Eigen::VectorXd pin_wrapper::comPosition()
{
    Eigen::Vector3d com;
    pinocchio::centerOfMass(*pmodel_, *data_, qd);
    com = data_->com[0];
    return com;
}

Eigen::MatrixXd pin_wrapper::comJacobian() const
{
    Eigen::MatrixXd Jcom;
    Jcom.setZero(3, pmodel_->nv);

    pinocchio::jacobianCenterOfMass(*pmodel_, *data_, qd);
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
        std::cout << jnames_[i] << "\t\t\t" << qd(i) << "\t\t\t\t" << qdotd(i) << std::endl;
}

void pin_wrapper::printActualJointData() const
{
    std::cout << "Joint name"
              << "\t\t\t"
              << "A. Joint Position"
              << "\t\t\t\t"
              << "A. Joint Velocity" << std::endl;

    for (int i = 0; i < jnames_.size(); ++i)
        std::cout << jnames_[i] << "\t\t\t" << qd(i) << "\t\t\t\t" << qdotd(i) << std::endl;
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
void pin_wrapper::setLinearTask(const std::string &frame_name, int task_type, Eigen::Vector3d des, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM" << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    Eigen::Vector3d v;
    Jac.resize(3, pmodel_->nv);

    if (task_type == 0)
    {
        Jac = linearJacobian(frame_name);
        // cout<<"Frame \n"<<frame_name<<endl;
        // cout<<"des \n"<<des<<endl;
        // cout<<"actual \n"<<linkPosition(frame_name)<<endl;

        v = (des - linkPosition(frame_name)) / dt;
    }
    else if (task_type == 2)
    {
        Jac = comJacobian();
        // cout<<"Frame \n"<<"CoM"<<endl;
        // cout<<"des \n"<<des<<endl;
        // cout<<"actual \n"<<comPosition()<<endl;
        v = (des - comPosition()) / dt;
    }
    cost += (Jac * qdotd - gain * v).squaredNorm() * weight;
    H += weight * Jac.transpose() * Jac;// + lm_damping*fmax(lm_damping, v.norm())   * I; //fmax(lm_damping, v.norm())
    h += (-weight * gain * v.transpose() * Jac).transpose();
}


void pin_wrapper::setAngularTask(const std::string &frame_name, int task_type, Eigen::Quaterniond des, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM" << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    Eigen::Vector3d v;
    Jac.resize(3, pmodel_->nv);

    Jac = angularJacobian(frame_name);
    // cout<<"Frame \n"<<frame_name<<endl;
    // cout<<"des \n"<<des.w()<<" "<<des.x()<<" "<<des.y()<<" "<<des.z() <<endl;
    // cout<<"actual \n"<<linkOrientation(frame_name).w()<<" "<<linkOrientation(frame_name).x()<<" "<<linkOrientation(frame_name).y()<<" "<<linkOrientation(frame_name).z() <<endl;
    v = logMap(des * (linkOrientation(frame_name)).inverse()) / dt;
    //v = (des.toRotationMatrix().eulerAngles(0, 1, 2) - linkOrientation(frame_name).toRotationMatrix().eulerAngles(0, 1, 2))/2
    //cout<<"Error "<<v<<endl;
    cost += (Jac * qdotd - gain * v).squaredNorm() * weight;
    H += weight * Jac.transpose() * Jac;// + lm_damping * fmax(lm_damping, v.norm()) * I;
    h += (-weight * gain * v.transpose() * Jac).transpose();
}
void pin_wrapper::setDOFTask(const std::string &joint_name, int task_type, double des, double weight, double gain, double dt)
{

    if (task_type > 3 || task_type < 0)
    {
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM / 3 for DOF " << std::endl;
        return;
    }

    Eigen::MatrixXd Jac;
    double v, qqdes, qq, qqdot;
    Jac.resize(1, pmodel_->nv);

    Jac.setZero();
  

    int idx = getJointId(joint_name);

    Jac(0,idx) = 1;
    qq = getQd(joint_name);
    qqdes = des;
    v = (qqdes - qq)/ dt;

    qqdot = getQdotd(joint_name);
    double cost__ = ( qqdot - gain * v)*( qqdot - gain * v) * weight;
    cost += cost__;
    std::cout<<"Joint "<<joint_name<<"Id "<<idx<<" Data "<<qq<<" "<<qqdot<<" Des "<<qqdes<<" Cost "<<cost__<<std::endl;
    std::cout<<"weight  "<<Jac<<"gain "<<v<<std::endl;
    H += weight * Jac.transpose() * Jac; //+ lm_damping * fmax(lm_damping, v*v) * I;
    h += (-weight * gain * v * Jac).transpose();
}

void pin_wrapper::addTasks(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask,  double dt)
{
    unsigned int i = 0;
    while (i < ltask.size())
    {
        setLinearTask(ltask[i].frame_name, ltask[i].task_type, ltask[i].des, ltask[i].weight, ltask[i].gain, dt);
        i++;

    }
    i = 0;
    while (i < atask.size())
    {
        setAngularTask(atask[i].frame_name, atask[i].task_type, atask[i].des, atask[i].weight, atask[i].gain, dt);
        i++;

    }
    i = 0;
    while (i < dtask.size())
    {
        setDOFTask(dtask[i].joint_name, dtask[i].task_type, dtask[i].des, dtask[i].weight, dtask[i].gain, dt);
        i++;
    }
}

Eigen::VectorXd pin_wrapper::inverseKinematics(std::vector<linearTask> ltask, std::vector<angularTask> atask,  std::vector<dofTask> dtask, double dt)
{

    unsigned int j = 0;
    //addTasks(ltask, atask, dt);


    while (j < 5000)
    {
        clearTasks();
        addTasks(ltask, atask, dtask, dt);
        lbq = gainC * (jointMinAngularLimits() - qd) / dt;
        ubq = gainC * (jointMaxAngularLimits() - qd) / dt;

        for (unsigned int i = 0; i < pmodel_->nv; ++i)
        {
            lb(i) = lbq(i) > lbdq(i) ? lbq(i) : lbdq(i);
            ub(i) = ubq(i) < ubdq(i) ? ubq(i) : ubdq(i);
        }

        //qdotd = H.colPivHouseholderQr().solve(-h);
        //std::cout << "Unconstrained Optimal Solution" << qdotd_ << std::endl;
        //cholesky.compute(H);
        //qdotd_ = cholesky.solve(-h);
        //std::cout << "Unconstrained Optimal Solution" << qdotd_ << std::endl;
        //std::cout << "-----" << std::endl;
        //L_choleksy = Eigen::MatrixXd(cholesky.matrixL());
        //qpmad::Solver::ReturnStatus status = solver.solve(qdotd, L_choleksy, h, lb, ub, A, Alb, Aub, solver_params);
        qpmad::Solver::ReturnStatus status = solver.solve(qdotd, H, h, lb, ub);
        qd = pinocchio::integrate(*pmodel_, qd, qdotd * dt);
       // cout<<"cost "<<fabs((cost - cost_)/cost_)<<endl;
        if( fabs((cost - cost_)/cost_) < 1.0e-6)
            break;
        forwardKinematics(qd, qdotd);
        j++;
    }   
    return qdotd;
}
