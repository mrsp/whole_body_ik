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

using namespace std;

class pin_wrapper
{

private:
    pinocchio::Model *pmodel_;
    pinocchio::Data *data_;
    std::vector<std::string> jnames_;
    Eigen::VectorXd qmin_, qmax_, dqmax_, q_, qdot_, qn;
    bool has_floating_base_;
    qpmad::Solver solver;
    qpmad::SolverParameters solver_params;
    double lm_damping = 1e-3;
    double gainC = 0.35;
    double dt = 0.01;
    Eigen::MatrixXd I;
    Eigen::VectorXd qdotd,qdotd_;
    Eigen::MatrixXd H;
    Eigen::VectorXd h;
    Eigen::MatrixXd A;
    Eigen::VectorXd Alb;
    Eigen::VectorXd Aub;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> cholesky;
    Eigen::MatrixXd L_choleksy;
    bool taskInit = false;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd qq;
    pin_wrapper(const std::string &model_name,
                const bool &has_floating_base, const bool &verbose = false);

    inline int ndof() const
    {
        return pmodel_->nq;
    }

    inline int ndofActuated() const
    {
        if (has_floating_base_)
            // Eliminate the Cartesian position and orientation (quaternion)
            return pmodel_->nq - 7;
        else
            return pmodel_->nq;
    }


    void updateJointConfig(std::map<std::string, double> qmap, std::map<std::string, double> qdotmap, double joint_std = 0);

    inline Eigen::Vector3d getLinearVelocityNoise(const std::string &frame_name)
    {
        return linearJacobian(frame_name) * qn;
    }
    
    inline Eigen::Vector3d getAngularVelocityNoise(const std::string &frame_name)
    {
        return angularJacobian(frame_name) * qn;
    }
    
    void mapJointNamesIDs(std::map<std::string, double> qmap, std::map<std::string, double> qdotmap);

    Eigen::MatrixXd geometricJacobian(const std::string &frame_name);

    inline Eigen::Vector3d getLinearVelocity(const std::string &frame_name)
    {
        return (linearJacobian(frame_name) * qdot_);
    }

    inline Eigen::Vector3d getAngularVelocity(const std::string &frame_name)
    {
        return (angularJacobian(frame_name) * qdot_);
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

    void setTask(const std::string &frame_name, int task_type, Eigen::Vector3d vdes, double weight, double gain);

    void addTask(Eigen::Vector3d vdes, Eigen::MatrixXd Jac,  double weight, double gain);
    
    inline void setdt(double dt_)
    {
        dt = dt_;
    }
    
    inline void setContraintGain(double gainC_)
    {
        gainC = gainC_;
    }
    
    Eigen::VectorXd inverseKinematics();
    
};
#endif
