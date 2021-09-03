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
using namespace Eigen;
struct linearTask
{
    std::string frame_name;
    int task_type;
    Eigen::Vector3d vdes;
    Eigen::Vector3d des;
    double weight;
    double gain;
};

struct angularTask
{
    std::string frame_name;
    int task_type;
    Eigen::Vector3d wdes;
    Eigen::Quaterniond qdes;
    double weight;
    double gain;
};
struct dofTask
{
    std::string joint_name;
    int task_type;
    double des;
    double weight;
    double gain;
};

struct humanoidTaskData
{
  VectorXd joint_states;
  std::vector<std::string> joint_names;
  Vector3d CoM_pos;
  Vector3d CoM_vel;
  Vector3d lfoot_pos;
  Vector3d rfoot_pos;
  Vector3d lhand_pos;
  Vector3d rhand_pos;
  Vector3d head_pos;
  Vector3d lfoot_linear_vel;
  Vector3d rfoot_linear_vel;
  Vector3d lhand_linear_vel;
  Vector3d rhand_linear_vel;
  Vector3d head_linear_vel;
  Vector3d lfoot_angular_vel;
  Vector3d rfoot_angular_vel;
  Vector3d lhand_angular_vel;
  Vector3d rhand_angular_vel;
  Vector3d head_angular_vel;
  Vector3d base_pos;
  Vector3d base_linear_vel;
  Vector3d base_angular_vel;
  Quaterniond base_orientation;
  Quaterniond lfoot_orientation;
  Quaterniond rfoot_orientation;
  Quaterniond lhand_orientation;
  Quaterniond rhand_orientation;
  Quaterniond head_orientation;
  std::string head_frame;
  std::string lfoot_frame;
  std::string rfoot_frame;
  std::string lhand_frame;
  std::string rhand_frame;
  std::string base_frame;
};


/** sinus cardinal: sin(x)/x
 * Code adapted from boost::math::detail::sinc
 */
template<typename T>
T sinc(const T x)
{
  constexpr T taylor_0_bound = std::numeric_limits<double>::epsilon();
  constexpr T taylor_2_bound = std::sqrt(taylor_0_bound);
  constexpr T taylor_n_bound = std::sqrt(taylor_2_bound);

  if(std::abs(x) >= taylor_n_bound)
  {
    return (std::sin(x) / x);
  }
  else
  {
    // approximation by taylor series in x at 0 up to order 0
    T result = static_cast<T>(1);

    if(std::abs(x) >= taylor_0_bound)
    {
      T x2 = x * x;

      // approximation by taylor series in x at 0 up to order 2
      result -= x2 / static_cast<T>(6);

      if(std::abs(x) >= taylor_2_bound)
      {
        // approximation by taylor series in x at 0 up to order 4
        result += (x2 * x2) / static_cast<T>(120);
      }
    }

    return (result);
  }
}

/**
 * Compute 1/sinc(x).
 * This code is inspired by boost/math/special_functions/sinc.hpp.
 */
template<typename T>
T sinc_inv(const T x)
{
  constexpr T taylor_0_bound = std::numeric_limits<T>::epsilon();
  constexpr T taylor_2_bound = std::sqrt(taylor_0_bound);
  constexpr T taylor_n_bound = std::sqrt(taylor_2_bound);

  // We use the 4th order taylor series around 0 of x/sin(x) to compute
  // this function:
  //
  //     x^2  7x^4
  // 1 + ── + ──── + O(x^6)
  //     6    360
  // this approximation is valid around 0.
  // if x is far from 0, our approximation is not valid
  // since x^6 becomes non negligible we use the normal computation of the function
  // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
  //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

  if(std::abs(x) >= taylor_n_bound)
  {
    return (x / std::sin(x));
  }
  else
  {
    // x is below taylor_n_bound so we don't care about the 6th order term of
    // the taylor series.
    // We set the 0 order term.
    T result = static_cast<T>(1);

    if(std::abs(x) >= taylor_0_bound)
    {
      // x is above the machine epsilon so x^2 is meaningful.
      double x2 = x * x;
      result += x2 / static_cast<T>(6);

      if(std::abs(x) >= taylor_2_bound)
      {
        // x is above the machine sqrt(epsilon) so x^4 is meaningful.
        result += static_cast<double>(7) * (x2 * x2) / static_cast<double>(360);
      }
    }

    return (result);
  }
}
inline Eigen::Vector3d rotationVelocity(const Eigen::Matrix3d & E_a_b)
{
  Eigen::Vector3d w;
  double acosV = (E_a_b(0, 0) + E_a_b(1, 1) + E_a_b(2, 2) - 1.) * 0.5;
  double theta = std::acos(std::min(std::max(acosV, -1.), 1.));

  w = Eigen::Vector3d(-E_a_b(2, 1) + E_a_b(1, 2), -E_a_b(0, 2) + E_a_b(2, 0), -E_a_b(1, 0) + E_a_b(0, 1));
  w *= sinc_inv(theta) * 0.5;

  return w;
}

inline Eigen::Vector3d rotationError(const Eigen::Matrix3d & E_a_b, const Eigen::Matrix3d & E_a_c)
{
  Eigen::Matrix3d E_b_c = E_a_c * E_a_b.transpose();
  return Eigen::Vector3d(E_a_b.transpose() * rotationVelocity(E_b_c));
}






class pin_wrapper
{

private:
    pinocchio::Model *pmodel_;
    pinocchio::Data *data_;
    std::vector<std::string> jnames_;
    Eigen::VectorXd qmin_, qmax_, dqmax_, q_, qdot_, qd, qdotd;
    bool has_floating_base;
    qpmad::Solver solver;
    qpmad::SolverParameters solver_params;
    double lm_damping;
    double gainC;
    double cost, cost_;
    Eigen::MatrixXd I;
    int iters;
    bool initialized;
    Eigen::MatrixXd A;
    Eigen::VectorXd Alb;
    Eigen::VectorXd Aub;
    Eigen::VectorXd lb, lbdq, lbq;
    Eigen::VectorXd ub, ubdq, ubq;
    Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> cholesky;
    Eigen::MatrixXd L_choleksy;
    void clearTasks();
    void setAngularTask(const std::string &frame_name, int task_type, Eigen::Vector3d wdes, Eigen::Quaterniond des, double weight, double gain, double dt);
    void addTasks(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask, double dt);
    void addReguralization();
    void forwardKinematics(Eigen::VectorXd pin_joint_pos, Eigen::VectorXd pin_joint_vel);
    Eigen::Vector3d logMap(Eigen::Quaterniond q);
    void setLinearTask(const std::string &frame_name, int task_type, Eigen::Vector3d vdes, Eigen::Vector3d des, double weight, double gain, double dt);
    void setDOFTask(const std::string &joint_name, int task_type, double des, double weight, double gain, double dt);
    bool jointDataReceived;

    Eigen::Vector3d vwb, omegawb, pwb;
    Eigen::Matrix3d Rwb;
    Eigen::Affine3d Twb;
    Eigen::Quaterniond qwb;

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

    void getJointData(const std::vector<std::string> &jnames,
                               Eigen::VectorXd &qvec,
                               Eigen::VectorXd &qdotvec);
    void getDesiredJointData(const std::vector<std::string> &jnames_,
                             std::vector<double> &qvec,
                             std::vector<double> &qdotvec);

    void getDesiredJointData(Eigen::VectorXd &qvec,
                             Eigen::VectorXd &qdotvec);

    void getDesiredJointData(const std::vector<std::string> &jnames_,
                                      Eigen::VectorXd &qvec,
                                      Eigen::VectorXd &qdotvec);
    std::vector<std::string> getJointNames();
    
    double getQ(const std::string &jname) const;
    double getQdot(const std::string &jname) const;
    double getQd(const std::string &jname) const;
    double getQdotd(const std::string &jname) const;
    void printActualJointData() const;
    void printDesiredJointData() const;
    void updateJointConfig(const std::vector<std::string> &jnames_,
                           const std::vector<double> &qvec,
                           const std::vector<double> &qdotvec);
                           
    void updateJointConfig(const std::vector<std::string> &jnames_,
                           const Eigen::VectorXd &qvec,
                           const Eigen::VectorXd &qdotvec);

    void updateJointConfig(Eigen::VectorXd q, Eigen::VectorXd dq);
    void mapJointNamesIDs(const std::vector<std::string> &jnames_,
                          const std::vector<double> &qvec,
                          const std::vector<double> &qdotvec);

    void mapJointNamesIDs(const std::vector<std::string> &jnames_,
                          const Eigen::VectorXd &qvec,
                          const Eigen::VectorXd &qdotvec);
    
    Eigen::VectorXd getGeneralizedCoordinates();

    Eigen::VectorXd getGeneralizedVelocities();

    Eigen::MatrixXd geometricJacobian(const std::string &frame_name);

    inline Eigen::Vector3d getLinearVelocity(const std::string &frame_name)
    {
        return (linearJacobian(frame_name) * qdot_);
    }

    inline Eigen::Vector3d getAngularVelocity(const std::string &frame_name)
    {
        return (angularJacobian(frame_name) * qdot_);
    }
    Eigen::VectorXd comVelocity();

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

    inline Eigen::Vector3d logMap(const Eigen::Matrix3d &R_)
    {
        Eigen::Vector3d w;
        double acosV = (R_(0, 0) + R_(1, 1) + R_(2, 2) - 1.) * 0.5;
        double theta = std::acos(std::min(std::max(acosV, -1.), 1.));

        w = Eigen::Vector3d(R_(2, 1) - R_(1, 2), R_(0, 2) - R_(2, 0), R_(1, 0) - R_(0, 1));
        w *= sinc_inv(theta) * 0.5;

        return w;
    }
    double sinc_inv(const double x)
    {
    constexpr double taylor_0_bound = std::numeric_limits<double>::epsilon();
    constexpr double taylor_2_bound = std::sqrt(taylor_0_bound);
    constexpr double taylor_n_bound = std::sqrt(taylor_2_bound);

    // We use the 4th order taylor series around 0 of x/sin(x) to compute
    // this function:
    //
    //     x^2  7x^4
    // 1 + ── + ──── + O(x^6)
    //     6    360
    // this approximation is valid around 0.
    // if x is far from 0, our approximation is not valid
    // since x^6 becomes non neglectable we use the normal computation of the function
    // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
    //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

    if(std::abs(x) >= taylor_n_bound)
    {
        return (x / std::sin(x));
    }
    else
    {
        // x is bellow taylor_n_bound so we don't care of the 6th order term of
        // the taylor series.
        // We set the 0 order term.
        double result = 1;

        if(std::abs(x) >= taylor_0_bound)
        {
        // x is above the machine epsilon so x^2 is meaningful.
        double x2 = x * x;
        result += x2 / 6;

        if(std::abs(x) >= taylor_2_bound)
        {
            // x is upper the machine sqrt(epsilon) so x^4 is meaningful.
            result += 7 * (x2 * x2) /360;
        }
        }

        return (result);
    }
    }




    Eigen::VectorXd inverseKinematics(std::vector<linearTask> ltask, std::vector<angularTask> atask, std::vector<dofTask> dtask, double dt);

    /** @fn Matrix3d wedge(Vector3d v)
	 * 	@brief Computes the skew symmetric matrix of a 3-D vector
	 *  @param v  3D Twist vector 
	 *  @return   3x3 skew symmetric representation
	 */
    inline Eigen::Matrix3d wedge(Eigen::Vector3d v)
    {
        Eigen::Matrix3d skew;
        skew = Eigen::Matrix3d::Zero();
        skew(0, 1) = -v(2);
        skew(0, 2) = v(1);
        skew(1, 2) = -v(0);
        skew(1, 0) = v(2);
        skew(2, 0) = -v(1);
        skew(2, 1) = v(0);

        return skew;
    }
    void setBaseToWorldState(Eigen::Vector3d pwb_, Eigen::Quaterniond qwb_);
    void setBaseWorldVelocity(Eigen::Vector3d vwb_, Eigen::Vector3d omegawb_);
    void setBaseToWorldTransform(Eigen::Affine3d Twb_);
};
#endif
