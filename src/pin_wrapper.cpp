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

    if (has_floating_base)
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
        //Do not insert "universe" joint
        if (jname.compare("universe") != 0)
        {
            jnames_.push_back(jname);
        }
    }

    qmin_.resize(jnames_.size());
    qmax_.resize(jnames_.size());
    dqmax_.resize(jnames_.size());
    qn.resize(jnames_.size());
    qn.setZero();
    qq.resize(jnames_.size());
    qq.setZero();
    qmin_ = pmodel_->lowerPositionLimit;
    qmax_ = pmodel_->upperPositionLimit;
    dqmax_ = pmodel_->velocityLimit;

    // If free-floating base, eliminate the "root_joint" when displaying
    if (has_floating_base_)
    {
        jnames_.erase(jnames_.begin());
        Eigen::VectorXd tmp;
        tmp = qmin_;
        qmin_ = tmp.tail(jnames_.size());
        tmp = qmax_;
        qmax_ = tmp.tail(jnames_.size());
        tmp = dqmax_;
        dqmax_ = tmp.tail(jnames_.size());
    }

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
    qdotd_.setZero(pmodel_->nv);

    //Not used in QP
    A.resize(0, 0);
    Alb.resize(0);
    Aub.resize(0);
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

void pin_wrapper::updateJointConfig(const std::vector<std::string> &jnames_,
                                    const std::vector<double> &qvec,
                                    const std::vector<double> &qdotvec,
                                    double joint_std)
{
    H.setZero(pmodel_->nv, pmodel_->nv);
    h.setZero(pmodel_->nv);
    taskInit = false;

    mapJointNamesIDs(jnames_, qvec, qdotvec);
    if (has_floating_base_)
    {
        // Change quaternion order: in Pinocchio it is
        // (x,y,z,w)
        Eigen::VectorXd qpin;
        qpin = q_;
        qpin[3] = q_[4];
        qpin[4] = q_[5];
        qpin[5] = q_[6];
        qpin[6] = q_[3];

        pinocchio::framesForwardKinematics(*pmodel_, *data_, qpin);
        pinocchio::computeJointJacobians(*pmodel_, *data_, qpin);
    }
    else
    {

        pinocchio::framesForwardKinematics(*pmodel_, *data_, q_);
        pinocchio::computeJointJacobians(*pmodel_, *data_, q_);
    }
    qn.setOnes();
    qn *= joint_std;
}

void pin_wrapper::setPoistionControl(double leak_rate)
{
    position_control = true;

    li = new LeakyIntegrator *[pmodel_->nv];
    for (unsigned int i = 0; i < pmodel_->nv; i++)
    {
        li[i] = new LeakyIntegrator();
        li[i]->setRate(leak_rate);
        li[i]->setInitialState(q_(i));
        li[i]->setSaturation(qmax_(i), qmin_(i));
    }
}

void pin_wrapper::mapJointNamesIDs(const std::vector<std::string> &jnames_,
                                   const std::vector<double> &qvec,
                                   const std::vector<double> &qdotvec)
{
    assert(qvec.size() == jnames_.size() && qdotvec.size() == jnames_.size());

    q_.resize(pmodel_->nq);
    qdot_.resize(pmodel_->nv);
    qq.resize(pmodel_->nv);

    for (int i = 0; i < jnames_.size(); i++)
    {
        int jidx = pmodel_->getJointId(jnames_[i]);
        int qidx = pmodel_->idx_qs[jidx];
        int vidx = pmodel_->idx_vs[jidx];

        //this value is equal to 2 for continuous joints
        if (pmodel_->nqs[jidx] == 2)
        {
            q_[qidx] = cos(qvec[i]);
            q_[qidx + 1] = sin(qvec[i]);
            qdot_[vidx] = qdotvec[i];
            qq[vidx] = qvec[i];
        }
        else
        {
            q_[qidx] = qvec[i];
            qdot_[vidx] = qdotvec[i];
            qq[vidx] = qvec[i];
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

        qvec[i] = qq[vidx];
        qdotvec[i] = qdot_[vidx];
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

double pin_wrapper::getQq(const std::string &jname) const
{
    int jidx = pmodel_->getJointId(jname);
    int vidx = pmodel_->idx_vs[jidx];

    return qq[vidx];
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

Eigen::MatrixXd pin_wrapper::geometricJacobian(const std::string &frame_name)
{
    try
    {
        pinocchio::Data::Matrix6x J(6, pmodel_->nv);
        J.fill(0);
        // Jacobian in pinocchio::LOCAL (link) frame
        pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);

        pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);

        if (has_floating_base_)
        {
            Eigen::MatrixXd Jg;
            Jg.resize(6, this->ndof());
            Jg.setZero();
            Eigen::Matrix3d Rbase;
            Rbase = quaternionToRotation(q_.segment(3, 4));
            Jg.topLeftCorner(3, 3) =
                Rbase.transpose() * data_->oMf[link_number].rotation() * J.block(0, 0, 3, 3);
            Jg.topRightCorner(3, ndofActuated()) =
                (data_->oMf[link_number].rotation()) * J.block(0, 6, 3, ndofActuated());
            Jg.bottomRightCorner(3, ndofActuated()) =
                (data_->oMf[link_number].rotation()) * J.block(3, 6, 3, ndofActuated());

            Eigen::MatrixXd T;
            T.resize(3, 4);
            T << -2.0 * q_(4), 2.0 * q_(3), -2.0 * q_(6), 2.0 * q_(5),
                -2.0 * q_(5), 2.0 * q_(6), 2.0 * q_(3), -2.0 * q_(4),
                -2.0 * q_(6), -2.0 * q_(5), 2.0 * q_(4), 2.0 * q_(3);
            Jg.block(0, 3, 3, 4) =
                data_->oMf[link_number].rotation() * J.block(0, 3, 3, 3) * Rbase.transpose() * T;
            Jg.block(3, 3, 3, 4) = T;

            return Jg;
        }
        else
        {
            // Transform Jacobians from pinocchio::LOCAL frame to base frame
            J.topRows(3) = (data_->oMf[link_number].rotation()) * J.topRows(3);
            J.bottomRows(3) = (data_->oMf[link_number].rotation()) * J.bottomRows(3);
            return J;
        }
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

Eigen::MatrixXd pin_wrapper::linearJacobian(const std::string &frame_name)
{
    pinocchio::Data::Matrix6x J(6, pmodel_->nv);
    J.fill(0);
    pinocchio::Model::FrameIndex link_number = pmodel_->getFrameId(frame_name);

    pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);
    try
    {
        if (has_floating_base_)
        {
            // Structure of J is:
            // [ Rworld_wrt_link*Rbase_wrt_world |
            //   Rworld_wrt_link*skew(Pbase_wrt_world-Plink_wrt_world)*R_base_wrt_world |
            //   Jq_wrt_link]
            Eigen::MatrixXd Jlin;
            Jlin.resize(3, this->ndof());
            Jlin.setZero();
            Eigen::Matrix3d Rbase;
            Rbase = quaternionToRotation(q_.segment(3, 4));
            Jlin.leftCols(3) =
                Rbase.transpose() * data_->oMf[link_number].rotation() * J.block(0, 0, 3, 3);
            Jlin.rightCols(ndofActuated()) =
                (data_->oMf[link_number].rotation()) * J.block(0, 6, 3, ndofActuated());

            Eigen::MatrixXd T;
            T.resize(3, 4);
            T << -2.0 * q_(4), 2.0 * q_(3), -2.0 * q_(6), 2.0 * q_(5),
                -2.0 * q_(5), 2.0 * q_(6), 2.0 * q_(3), -2.0 * q_(4),
                -2.0 * q_(6), -2.0 * q_(5), 2.0 * q_(4), 2.0 * q_(3);
            Jlin.middleCols(3, 4) =
                data_->oMf[link_number].rotation() * J.block(0, 3, 3, 3) * Rbase.transpose() * T;

            return Jlin;
        }
        else
        {
            // Transform Jacobian from pinocchio::LOCAL frame to base frame
            return (data_->oMf[link_number].rotation()) * J.topRows(3);
        }
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
        if (has_floating_base_)
        {

            Eigen::MatrixXd Jang;
            Jang.resize(3, this->ndof());
            Jang.setZero();
            Eigen::Vector4d q;

            // Jacobian in global frame
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);

            // The structure of J is: [0 | Rot_ff_wrt_world | Jq_wrt_world]
            Jang.rightCols(ndofActuated()) = J.block(3, 6, 3, ndofActuated());
            q = rotationToQuaternion(J.block(3, 3, 3, 3));
            Jang.middleCols(3, 4) << -2.0 * q(1), 2.0 * q(0), -2.0 * q(3), 2.0 * q(2),
                -2.0 * q(2), 2.0 * q(3), 2.0 * q(0), -2.0 * q(1),
                -2.0 * q(3), -2.0 * q(2), 2.0 * q(1), 2.0 * q(0);
            return Jang;
        }
        else
        {

            // Jacobian in pinocchio::LOCAL frame
            pinocchio::getFrameJacobian(*pmodel_, *data_, link_number, pinocchio::LOCAL, J);

            // Transform Jacobian from pinocchio::LOCAL frame to base frame
            return (data_->oMf[link_number].rotation()) * J.bottomRows(3);
        }
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

    if (has_floating_base_)
    {
        // Change quaternion order: in oscr it is (w,x,y,z) and in Pinocchio it is
        // (x,y,z,w)
        Eigen::VectorXd qpin;

        qpin = q_;
        qpin[3] = q_[4];
        qpin[4] = q_[5];
        qpin[5] = q_[6];
        qpin[6] = q_[3];
        //Eigen::Vector3d com = pinocchio::centerOfMass(*pmodel_, *data_, qpin);
        //std::cout << qpin.head(7).transpose() << std::endl;
        pinocchio::centerOfMass(*pmodel_, *data_, qpin);

        // Eigen::Matrix3d Rbase; Rbase = quaternionToRotation(q_.segment(3,4));
        // com = Rbase*data_->com[0];
        com = data_->com[0];
    }
    else
    {
        //Eigen::Vector3d com = pinocchio::centerOfMass(*pmodel_, *data_, q_);
        pinocchio::centerOfMass(*pmodel_, *data_, q_);
        com = data_->com[0];
    }

    return com;
}

Eigen::MatrixXd pin_wrapper::comJacobian() const
{
    Eigen::MatrixXd Jcom;
    Jcom.setZero(3, pmodel_->nv);

    Jcom = pinocchio::jacobianCenterOfMass(*pmodel_, *data_, q_);

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
              << "Joint Position"
              << "\t\t\t\t"
              << "Joint Velocity" << std::endl;

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

void pin_wrapper::setTask(const std::string &frame_name, int task_type, Eigen::Vector3d vdes, double weight, double gain)
{
    if (task_type == 0)
    {
        taskInit = true;
        Eigen::MatrixXd Jac;
        Jac.resize(3, pmodel_->nv);
        Jac = linearJacobian(frame_name);
        addTask(vdes, Jac, weight, gain);
    }
    else if (task_type == 1)
    {
        taskInit = true;
        Eigen::MatrixXd Jac;
        Jac.resize(3, pmodel_->nv);
        Jac = angularJacobian(frame_name);
        addTask(vdes, Jac, weight, gain);
    }
    else if (task_type == 2)
    {
        taskInit = true;
        Eigen::MatrixXd Jac;
        Jac.resize(3, pmodel_->nv);
        Jac = comJacobian();
        addTask(vdes, Jac, weight, gain);
    }
    else
    {
        taskInit = false;
        std::cout << "Wrong Task Type: 0 for linear / 1 for angular / 2 for CoM" << std::endl;
        return;
    }
}

void pin_wrapper::addTask(Eigen::Vector3d vdes, Eigen::MatrixXd Jac, double weight, double gain)
{
    //std::cout<<" H " <<H<<std::endl;
    //std::cout<<" h " <<h<<std::endl;

    Eigen::Vector3d res = vdes - Jac * qdot_;
    H += weight * Jac.transpose() * Jac + lm_damping * fmax(lm_damping, res.norm()) * I;
    h += (-weight * gain * vdes.transpose() * Jac).transpose();
}

Eigen::VectorXd pin_wrapper::inverseKinematics(double dt)
{

    if (taskInit)
    {
        lbq = gainC * (jointMinAngularLimits() - qq) / dt;
        ubq = gainC * (jointMaxAngularLimits() - qq) / dt;

        for (unsigned int i = 0; i < pmodel_->nv; ++i)
        {
            lb(i) = lbq(i) > lbdq(i) ? lbq(i) : lbdq(i);
            ub(i) = ubq(i) < ubdq(i) ? ubq(i) : ubdq(i);
        }

        //qdotd_ = H.colPivHouseholderQr().solve(-h);
        //std::cout << "Unconstrained Optimal Solution" << qdotd_ << std::endl;
        //cholesky.compute(H);
        //qdotd_ = cholesky.solve(-h);
        //std::cout << "Unconstrained Optimal Solution" << qdotd_ << std::endl;
        //std::cout << "-----" << std::endl;
        //L_choleksy = Eigen::MatrixXd(cholesky.matrixL());
        //qpmad::Solver::ReturnStatus status = solver.solve(qdotd, L_choleksy, h, lb, ub, A, Alb, Aub, solver_params);
        qpmad::Solver::ReturnStatus status = solver.solve(qdotd, H, h, lb, ub);

        if (position_control)
        {
            //Leaky Integration
            for (unsigned int i = 0; i < pmodel_->nv; i++)
            {
                li[i]->add(qdotd(i), dt);
                qd(i) = li[i]->eval();
                i++;
            }
        }
        return qdotd;
    }
    else
    {
        std::cout << "No Tasks Defined to solve for" << std::endl;
        qdotd.setZero(pmodel_->nv);
        return qdotd;
    }
}
