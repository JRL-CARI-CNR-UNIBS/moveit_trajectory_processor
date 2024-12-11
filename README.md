# moveit_trajectory_processor

The `moveit_trajectory_processor` package offers a class that extends [`trajectories_processors_lib`](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib.git) by integrating trajectory processing algorithms from MoveIt. For more information, refer to the [MoveIt Time Parameterization tutorial](https://moveit.github.io/moveit_tutorials/doc/time_parameterization/time_parameterization_tutorial.html).

### Overview

The core of this package is the `MoveitTrajectoryProcessor` class, defined in the `moveit_trajectory_processor.h` file. This class inherits from the [`SplineTrajectoryProcessor`](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/blob/master/include/openmore/trajectories_processors/spline_trajectory_processor.h) class and implements the `computeTrj()` function using one of three available MoveIt algorithms:

- **Iterative Time Parametrization (ITP)**
- **Iterative Spline Parametrization (ISP)**
- **Time Optimal Trajectory Generation (TOTG)**

The `MoveitTrajectoryProcessor` class computes the timing law for a given path using the selected MoveIt algorithm and then interpolates the computed trajectory using a spline of the specified order.

### Usage Example

Below is a basic example demonstrating how to use the `MoveitTrajectoryProcessor` class:

1. **Include the class definition:**
   ```cpp
   #include <moveit_trajectory_processor/moveit_trajectory_processor.h>
   ```

2. **Create and configure the trajectory processor:**
   ```cpp
   #include <moveit/robot_model_loader/robot_model_loader.h>
   #include <moveit/move_group_interface/move_group_interface.h>

   // Initialization

   using namespace openmore;
   std::string group_name = "your_group_name"; // MoveIt group name
   std::string param_ns = "your_param_ns"; // Namespace for retrieving parameters via the cnr_param library
   std::string path_to_logger_conf_file = "some_path"; // Path to the cnr_logger configuration file

   cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("logger_example", path_to_logger_conf_file);

   // MoveIt section

   moveit::planning_interface::MoveGroupInterface move_group(group_name);
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
   const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
   std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
   unsigned int dof = joint_names.size();

   // Initialize all members of constraints to appropriate default values
   
   KinodynamicConstraintsPtr constraints = std::make_shared<KinodynamicConstraints>();

   constraints->min_pos_ = Eigen::VectorXd::Constant(dof, -std::numeric_limits<double>::infinity());
   constraints->max_pos_ = Eigen::VectorXd::Constant(dof,  std::numeric_limits<double>::infinity());
   constraints->min_vel_ = Eigen::VectorXd::Constant(dof, -std::numeric_limits<double>::infinity());
   constraints->max_vel_ = Eigen::VectorXd::Constant(dof,  std::numeric_limits<double>::infinity());
   constraints->min_acc_ = Eigen::VectorXd::Constant(dof, -std::numeric_limits<double>::infinity());
   constraints->max_acc_ = Eigen::VectorXd::Constant(dof,  std::numeric_limits<double>::infinity());
   constraints->min_eff_ = Eigen::VectorXd::Constant(dof, -std::numeric_limits<double>::infinity());
   constraints->max_eff_ = Eigen::VectorXd::Constant(dof,  std::numeric_limits<double>::infinity());

   for (unsigned int idx = 0; idx < dof; idx++)
   {
     const robot_model::VariableBounds& bounds = robot_model->getVariableBounds(joint_names.at(idx));

     if (bounds.position_bounded_)
     {
       constraints->min_pos_(idx) = bounds.min_position_;
       constraints->max_pos_(idx) = bounds.max_position_;
     }
     if (bounds.velocity_bounded_)
     {
       constraints->min_vel_(idx) = bounds.min_velocity_;
       constraints->max_vel_(idx) = bounds.max_velocity_;
     }
     if (bounds.acceleration_bounded_)
     {
       constraints->min_acc_(idx) = bounds.min_acceleration_;
       constraints->max_acc_(idx) = bounds.max_acceleration_;
     }
   }

    // Define the spline order
    auto spline_order = MoveitTrajectoryProcessor::spline_order_t::THREE;

    // Select the algorithm for timing-law computation
    auto moveit_alg = MoveitTrajectoryProcessor::moveit_alg_t::ISP;

    MoveitTrajectoryProcessorPtr trajectory_processor = std::make_shared<MoveitTrajectoryProcessor>(
          constraints, param_ns, logger, spline_order, moveit_alg, group_name, robot_model);
   ```

This example sets up a trajectory processor that uses the Iterative Spline Parametrization (ISP) algorithm from MoveIt. You can easily modify this template to use different algorithms or customize it further according to your application needs.
