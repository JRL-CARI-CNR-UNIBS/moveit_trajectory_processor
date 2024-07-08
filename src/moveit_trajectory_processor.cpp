#include <moveit_trajectory_processor/moveit_trajectory_processor.h>

namespace trajectories_processors
{

void fromEigen2Vector(const Eigen::VectorXd& eigen, std::vector<double> vector)
{
  vector.clear();
  vector.resize(eigen.rows());
  Eigen::VectorXd::Map(&vector[0], eigen.rows()) = eigen;
}

moveit::core::RobotStatePtr fromWaypoint2State(const Eigen::VectorXd& waypoint,
                                               const moveit::core::RobotStateConstPtr& robot_state,
                                               const std::string& group_name)
{
  moveit::core::RobotStatePtr state = std::make_shared<moveit::core::RobotState>(*robot_state);
  state->setJointGroupPositions(group_name,waypoint);
  state->update();

  return state;
}

bool fromWaypoints2States(const std::vector<Eigen::VectorXd>& waypoints,
                          const moveit::core::RobotStateConstPtr& robot_state,
                          const std::string& group_name,
                          std::vector<moveit::core::RobotStatePtr>& states)
{
  states.clear();
  states.reserve(waypoints.size());

  for(const auto& waypoint: waypoints)
  {
    auto state = fromWaypoint2State(waypoint,robot_state,group_name);
    states.push_back(state);
  }

  return true;
}

bool MoveitTrajectoryProcessor::computeTrj()
{
  RobotStatePtr initial_state;
  return computeTrj(initial_state);
}

bool MoveitTrajectoryProcessor::computeTrj(const RobotStatePtr& initial_state)
{
  RobotStatePtr final_state;
  return computeTrj(initial_state,final_state);
}

bool MoveitTrajectoryProcessor::computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state)
{
  if(path_.empty())
  {
    CNR_ERROR(logger_,"Path not defined, cannot compute time law");
    return false;
  }

  if(path_.size() == 1)
  {
    CNR_ERROR(logger_,"Path is a single waypoint defined, cannot compute time law");
    return false;
  }

  robot_state::RobotStatePtr robot_state = std::make_shared<robot_state::RobotState>(robot_model_);

  std::vector<moveit::core::RobotStatePtr> states_vector;
  if(not fromWaypoints2States(path_,robot_state,group_name_,states_vector))
    return false;

  robot_trajectory::RobotTrajectoryPtr trj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_);

  for(unsigned int j=0; j<path_.size();j++)
  {
    if(j==0 && (initial_state != nullptr))
    {
      states_vector.at(j)->setJointGroupPositions    (group_name_,initial_state->pos_);
      states_vector.at(j)->setJointGroupVelocities   (group_name_,initial_state->vel_);
      states_vector.at(j)->setJointGroupAccelerations(group_name_,initial_state->acc_);
    }
    else if(j==(path_.size()-1) && (final_state != nullptr))
    {
      states_vector.at(j)->setJointGroupPositions    (group_name_,final_state->pos_);
      states_vector.at(j)->setJointGroupVelocities   (group_name_,final_state->vel_);
      states_vector.at(j)->setJointGroupAccelerations(group_name_,final_state->acc_);
    }
    trj->addSuffixWayPoint(states_vector.at(j),0.001);
  }

  //Time parametrization
  trajectory_processing::TimeParameterizationPtr tp;
  if(moveit_alg_ == moveit_alg_t::ITP)
    tp = std::make_shared<trajectory_processing::IterativeParabolicTimeParameterization>();
  else if(moveit_alg_ == moveit_alg_t::ISP)
    tp = std::make_shared<trajectory_processing::IterativeSplineParameterization>();
  else if(moveit_alg_ == moveit_alg_t::TOTG)
    tp = std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>();

  tp->computeTimeStamps(*trj);

  moveit_msgs::RobotTrajectory trj_msg;
  trj->getRobotTrajectoryMsg(trj_msg);

  trj_.clear();
  for(const auto& trj_point:trj_msg.joint_trajectory.points)
  {
    TrjPointPtr pt = std::make_shared<TrjPoint>();

    pt->state_->pos_ = std::move(trj_point.positions);
    pt->state_->vel_ = std::move(trj_point.velocities);
    pt->state_->acc_ = std::move(trj_point.accelerations);
    pt->state_->eff_ = std::move(trj_point.effort);
    pt->time_from_start_ = std::move(trj_point.time_from_start.toSec());

    trj_.push_back(pt);
  }

  return true;
}

bool MoveitTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::string &group_name, const moveit::core::RobotModelPtr &robot_model)
{
  SplineTrajectoryProcessor::init(constraints,param_ns,logger);

  group_name_ = group_name;
  robot_model_ = robot_model;

  std::string moveit_alg;
  std::string what, full_param_name = param_ns+"/moveit_alg";
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, moveit_alg, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      return false;
    }
    else
    {
      std::transform(moveit_alg.begin(), moveit_alg.end(), moveit_alg.begin(), ::toupper);

      if(moveit_alg == std::string("ITP"))
        moveit_alg_ = moveit_alg_t::ITP;
      else if(moveit_alg == std::string("ISP"))
        moveit_alg_ = moveit_alg_t::ISP;
      else if(moveit_alg == std::string("TOTG"))
        moveit_alg_ = moveit_alg_t::TOTG;
      else
      {
        CNR_ERROR(logger_,moveit_alg<<" algorithm not available");
        return false;
      }
    }
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }

  return true;
}
bool MoveitTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path, const std::string &group_name, const moveit::core::RobotModelPtr &robot_model)
{
  SplineTrajectoryProcessor::init(constraints,param_ns,logger,path);

  group_name_ = group_name;
  robot_model_ = robot_model;

  std::string moveit_alg;
  std::string what, full_param_name = param_ns+"/moveit_alg";
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, moveit_alg, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      return false;
    }
    else
    {
      std::transform(moveit_alg.begin(), moveit_alg.end(), moveit_alg.begin(), ::toupper);

      if(moveit_alg == std::string("ITP"))
        moveit_alg_ = moveit_alg_t::ITP;
      else if(moveit_alg == std::string("ISP"))
        moveit_alg_ = moveit_alg_t::ISP;
      else if(moveit_alg == std::string("TOTG"))
        moveit_alg_ = moveit_alg_t::TOTG;
      else
      {
        CNR_ERROR(logger_,moveit_alg<<" algorithm not available");
        return false;
      }
    }
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }

  return true;
}
}
