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
   #include <moveit/move_group_interface/move_group_interface.h>

   // Initialization

   using namespace openmore;
   std::string group_name = "your_group_name"; // MoveIt group name
   std::string param_ns = "your_param_ns"; // Namespace for retrieving parameters via the cnr_param library
   std::string path_to_logger_conf_file = "some_path"; // Path to the cnr_logger configuration file

   cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("logger_example", path_to_logger_conf_file);

   moveit::planning_interface::MoveGroupInterface move_group(group_name);
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

   KinodynamicConstraintsPtr constraints = std::make_shared<KinodynamicConstraints>();
   constraints->max_pos_ = max_pos; // max_pos is an Eigen::VectorXd
   constraints->min_pos_ = min_pos; // min_pos is an Eigen::VectorXd
   constraints->max_vel_ = max_vel; // max_vel is an Eigen::VectorXd
   constraints->min_vel_ = min_vel; // min_vel is an Eigen::VectorXd
   constraints->max_acc_ = max_acc; // max_acc is an Eigen::VectorXd
   constraints->min_acc_ = min_acc; // min_acc is an Eigen::VectorXd
   constraints->max_eff_ = max_eff; // max_eff is an Eigen::VectorXd
   constraints->min_eff_ = min_eff; // min_eff is an Eigen::VectorXd

   // Define the spline order
   auto spline_order = MoveitTrajectoryProcessor::spline_order_t::THREE;

   // Select the algorithm for timing-law computation
   auto moveit_alg = MoveitTrajectoryProcessor::moveit_alg_t::ISP;

   MoveitTrajectoryProcessorPtr trajectory_processor = std::make_shared<MoveitTrajectoryProcessor>(
         constraints, param_ns, logger, spline_order, moveit_alg, group_name, robot_model);
   ```

This example sets up a trajectory processor that uses the Iterative Spline Parametrization (ISP) algorithm from MoveIt. You can easily modify this template to use different algorithms or customize it further according to your application needs.
