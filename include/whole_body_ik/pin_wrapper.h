/*
 * humanoid_state_publisher - a ros package to generate necessary data for humanoid robots
 *
 * Copyright 2017-2020 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PINWRAP_H__
#define __PINWRAP_H__
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
#include <whole_body_ik/LeakyIntegrator.h>
using namespace std;

struct linearTask {
    std::string frame_name;
    int task_type;
    Eigen::Vector3d des;
    double weight;
    double gain;
};

struct angularTask {
    std::string frame_name;
    int task_type;
    Eigen::Quaterniond des;
    double weight;
    double gain;
};
struct dofTask {
    std::string joint_name;
    int task_type;
    double des;
    double weight;
    double gain;
};
class pin_wrapper
{

private:
    pinocchio::Model *pmodel_;
    pinocchio::Data *data_;
    std::vector<std::string> jnames_;
    Eigen::VectorXd qmin_, qmax_, dqmax_, q_, qd, qdotd;
    bool has_floating_base_;
    qpmad::Solver solver;
    qpmad::SolverParameters solver_params;
    double lm_damping;
    double gainC;
    double cost, cost_;
    Eigen::MatrixXd I;
    int iters;
    Eigen::MatrixXd A;
    Eigen::VectorXd Alb;
    Eigen::VectorXd Aub;
    Eigen::VectorXd lb, lbdq, lbq;
    Eigen::VectorXd ub, ubdq, ubq;
    Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> cholesky;
    Eigen::MatrixXd L_choleksy;
    void clearTasks();
    void setAngularTask(const std::string &frame_name, int task_type, Eigen::Quaterniond des, double weight, double gain, double dt);
    void addTasks(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask,  double dt);
    void forwardKinematics(Eigen::VectorXd pin_joint_pos, Eigen::VectorXd pin_joint_vel);
    Eigen::Vector3d logMap(Eigen::Quaterniond q);
    void setLinearTask(const std::string &frame_name, int task_type, Eigen::Vector3d des, double weight, double gain, double dt);
    void setDOFTask(const std::string &joint_name, int task_type, double des, double weight, double gain, double dt);


public:
    Eigen::MatrixXd H;
    Eigen::VectorXd h;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd qq;
    pin_wrapper(const std::string &model_name, const bool &has_floating_base, const bool &verbose = false);

    inline int ndof() const
    {
        return pmodel_->nq;
    }

    inline int ndofActuated() const
    {
            return pmodel_->nv;
    }

    void getJointData(const std::vector<std::string> &jnames_,
                      std::vector<double> &qvec,
                      std::vector<double> &qdotvec);
    void getDesiredJointData(const std::vector<std::string> &jnames_,
                             std::vector<double> &qvec,
                             std::vector<double> &qdotvec);
    double getQq(const std::string &jname) const;

    double getQdotd(const std::string &jname) const;

    double getQd(const std::string &jname) const;
    int getJointId(const std::string &jname) const;
    void printActualJointData() const;
    void printDesiredJointData() const;
    void updateJointConfig(const std::vector<std::string> &jnames_,
                           const std::vector<double> &qvec,
                           const std::vector<double> &qdotvec,
                           double joint_std = 0);

   
    void mapJointNamesIDs(const std::vector<std::string> &jnames_,
                          const std::vector<double> &qvec,
                          const std::vector<double> &qdotvec);

    Eigen::MatrixXd geometricJacobian(const std::string &frame_name);

    inline Eigen::Vector3d getLinearVelocity(const std::string &frame_name)
    {
        return (linearJacobian(frame_name) * qdotd);
    }

    inline Eigen::Vector3d getAngularVelocity(const std::string &frame_name)
    {
        return (angularJacobian(frame_name) * qdotd);
    }

    Eigen::Vector3d linkPosition(const std::string &frame_name);

    Eigen::Quaterniond linkOrientation(const std::string &frame_name);

    Eigen::VectorXd linkPose(const std::string &frame_name);

    Eigen::MatrixXd linearJacobian(const std::string &frame_name);

    Eigen::MatrixXd angularJacobian(const std::string &frame_name);

    Eigen::VectorXd comPosition();

    Eigen::MatrixXd comJacobian() const;

    inline std::vector<std::string> jointNames() const
    {
        return jnames_;
    }

    inline Eigen::VectorXd jointMaxAngularLimits() const
    {
        return qmax_;
    }

    inline Eigen::VectorXd jointMinAngularLimits() const
    {
        return qmin_;
    }

    inline Eigen::VectorXd jointVelocityLimits() const
    {
        return dqmax_;
    }

    inline void printJointNames() const
    {
        std::cout << *pmodel_ << std::endl;
    }

    void printJointLimits() const;

    inline double sgn(const double &x)
    {
        if (x >= 0)
            return 1.0;
        else
            return -1.0;
    }

    Eigen::Vector4d rotationToQuaternion(const Eigen::Matrix3d &R);

    Eigen::Matrix3d quaternionToRotation(const Eigen::Vector4d &q);

    Eigen::VectorXd inverseKinematics(std::vector<linearTask> ltask, std::vector<angularTask> atask,std::vector<dofTask> dtask, double dt);
};
#endif
