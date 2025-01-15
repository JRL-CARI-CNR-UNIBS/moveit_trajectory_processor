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

namespace openmore
{
/**
 * @file MoveitTrajectoryProcessor.h
 * @brief Contains the declaration of the MoveitTrajectoryProcessor class.
 *
 * Note: This class does not use the kinodynamic constraints provided as input but relies on the ones provided by Moveit.
 */
class MoveitTrajectoryProcessor;
typedef std::shared_ptr<MoveitTrajectoryProcessor> MoveitTrajectoryProcessorPtr;

/**
 * @brief Processes trajectories using Moveit algorithms and provides spline-based interpolation.
 */
class MoveitTrajectoryProcessor: public virtual SplineTrajectoryProcessor
{
public:

  /**
   * @brief Enum for selecting the Moveit algorithm for trajectory planning.
   * - ITP: Iterative Time Parameterization.
   * - ISP: Iterative Spline Parameterization.
   * - TOTG: Time Optimal Trajectory Generation.
   */
  enum class moveit_alg_t
  {
    ITP,  /**< Iterative Time Parameterization. */
    ISP,  /**< Iterative Spline Parameterization. */
    TOTG  /**< Time Optimal Trajectory Generation. */
  };

protected:

  /**
   * @brief The selected Moveit algorithm (default: ISP).
   */
  moveit_alg_t moveit_alg_ = moveit_alg_t::ISP;

  /**
   * @brief The name of the robot group for which the trajectory is processed.
   */
  std::string group_name_;

  /**
   * @brief A pointer to the Moveit RobotModel.
   */
  robot_model::RobotModelPtr robot_model_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   * Requires a call to init() aftwrwards.
   */
  MoveitTrajectoryProcessor():
    SplineTrajectoryProcessor(){}

  /**
   * @brief Constructor.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The parameter namespace.
   * @param logger The logger instance.
   * @param group_name The name of the robot group.
   * @param robot_model A pointer to the Moveit RobotModel.
   */
  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger),group_name_(group_name),robot_model_(robot_model){}

  /**
   * @brief Constructor with a predefined path.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The parameter namespace.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   * @param group_name The name of the robot group.
   * @param robot_model A pointer to the Moveit RobotModel.
   */
  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger,path),group_name_(group_name),robot_model_(robot_model){}

  /**
   * @brief Constructor with a specified spline order and Moveit algorithm.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The parameter namespace.
   * @param logger The logger instance.
   * @param spline_order The spline interpolation order.
   * @param moveit_alg The Moveit algorithm for trajectory planning.
   * @param group_name The name of the robot group.
   * @param robot_model A pointer to the Moveit RobotModel.
   */
  MoveitTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const spline_order_t& spline_order,
                            const moveit_alg_t& moveit_alg,
                            const std::string& group_name,
                            const robot_model::RobotModelPtr& robot_model):
    SplineTrajectoryProcessor(constraints,param_ns,logger,spline_order),moveit_alg_(moveit_alg),group_name_(group_name),robot_model_(robot_model){}

  /**
   * @brief Constructor with a predefined path, spline order, and Moveit algorithm.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The parameter namespace.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   * @param spline_order The spline interpolation order.
   * @param moveit_alg The Moveit algorithm for trajectory planning.
   * @param group_name The name of the robot group.
   * @param robot_model A pointer to the Moveit RobotModel.
   */
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
   * @brief Initializes the MoveitTrajectoryProcessor object with a predefined path.
   *
   * This function should be called when using the default constructor. It sets up the robot group and Moveit algorithms.
   *
   * @param constraints The kinodynamic constraints.
   * @param param_ns The parameter namespace.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   * @param group_name The name of the robot group.
   * @param robot_model A pointer to the Moveit RobotModel.
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::string& group_name, const robot_model::RobotModelPtr& robot_model);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path, const std::string& group_name, const robot_model::RobotModelPtr& robot_model);

  /**
   * @brief Sets the Moveit algorithm and clears the existing trajectory.
   * @param moveit_alg The Moveit algorithm to set.
   */
  void setMoveitAlg(const moveit_alg_t& moveit_alg)
  {
    moveit_alg_ = moveit_alg;
    trj_.clear();
  }

  /**
   * @brief Computes the trajectory using the selected Moveit algorithm.
   *
   * This function creates a trajectory based on the provided initial and final states and computes the timing using the selected Moveit algorithm.
   *
   * @param initial_state The initial state of the robot.
   * @param final_state The final state of the robot.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state) override;
  using TrajectoryProcessorBase::computeTrj; /**< Brings other overloads of the interpolate function into the scope. */
};
}
