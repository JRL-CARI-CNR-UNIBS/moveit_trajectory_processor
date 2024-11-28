/*
Copyright (c) 2024

JRL-CARI CNR-STIIMA/UNIBS
Cesare Tonola, c.tonola001@unibs.it
Manuel Beschi, manuel.beschi@unibs.it

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <openmore/trajectories_processors/spline_trajectory_processor.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

/**
 * @file MoveitTrajectoryProcessor.h
 * @brief Contains the declaration of the MoveitTrajectoryProcessor class.
 * Note that this class does not consider the KinoDynamicConstraints given as input but uses the ones provided by Moveit.
 */

namespace openmore
{
class MoveitTrajectoryProcessor;
typedef std::shared_ptr<MoveitTrajectoryProcessor> MoveitTrajectoryProcessorPtr;

/**
 * @brief The MoveitTrajectoryProcessor class processes trajectories using Moveit algorithms and interpolates with splines of different orders.
 */

class MoveitTrajectoryProcessor: public SplineTrajectoryProcessor
{
public:
  /**
   * @brief Enum for selecting the moveit algorithm for trajectory planning.
   *        - ITP: iterative time parametrization
   *        - ISP: iterative spline parametrization
   *        - TOTG: time optimal trajectory generation
   */
  enum class moveit_alg_t {ITP, ISP, TOTG};

protected:
  /**
   * @brief The selected moveit algorithm, default: ISP.
   */
  moveit_alg_t moveit_alg_ = moveit_alg_t::ISP;

  /**
   * @brief group_name_ The name of the robot group.
   */
  std::string group_name_;

  /**
   * @brief robot_model_ pointer to the Moveit RobotModel.
   */
  robot_model::RobotModelPtr robot_model_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructors.
   */
  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger),group_name_(group_name),robot_model_(robot_model){}

  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger,path),group_name_(group_name),robot_model_(robot_model){}

  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const spline_order_t& spline_order,
                            const moveit_alg_t& moveit_alg,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger,spline_order),moveit_alg_(moveit_alg),group_name_(group_name),robot_model_(robot_model){}

  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const spline_order_t& spline_order,
                            const moveit_alg_t& moveit_alg,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger,path,spline_order),moveit_alg_(moveit_alg),group_name_(group_name),robot_model_(robot_model){}

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @param group_name The robot group name.
   * @param robot_model A Moveit RobotModel pointer to the robot model.
   * @return true if successfull
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::string& group_name, const robot_model::RobotModelPtr& robot_model);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path, const std::string& group_name, const robot_model::RobotModelPtr& robot_model);

  /**
   * @brief Sets the moveit algorithm and clears the trajectory.
   * @param moveit_alg The moveit algorithm to set.
   */
  void setMoveitAlg(const moveit_alg_t& moveit_alg)
  {
    moveit_alg_ = moveit_alg;
    trj_.clear();
  }

  /**
   * @brief Function to compute the trajectory.
   * @param initial_state The initial robot state.
   * @param final_state The final robot state.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrj() override;
  virtual bool computeTrj(const RobotStatePtr& initial_state) override;
  virtual bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state) override;
};
}
