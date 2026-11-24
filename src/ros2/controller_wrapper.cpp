/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the controller wrapper for ROS
 */

// WoLF
#include <wolf_controller/controller_wrapper.h>
#include <wolf_controller_core/state_machine.h>

// System
#include <algorithm>
#include <cmath>
#include <functional>


ControllerRosWrapper::ControllerRosWrapper(rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node, wolf_controller::ControllerCore* const controller_ptr)
{
  controller_ = controller_ptr;

  // Init
  base_rpy_.fill(0.0);
  base_rpy_prev_.fill(0.0);

  // Set some ROS2 params
  controller_node->declare_parameter<std::string>("robot_base_name", controller_ptr->getRobotModel()->getBaseLinkName());
  controller_node->declare_parameter<std::vector<std::string>>("robot_foot_names", controller_ptr->getRobotModel()->getFootNames());
  controller_node->declare_parameter<std::vector<std::string>>("robot_arm_names", controller_ptr->getRobotModel()->getArmNames());
  controller_node->declare_parameter<std::string>("robot_imu_name", controller_ptr->getRobotModel()->getImuSensorName());
  if (!controller_node->has_parameter("mpc_contract_version"))
    controller_node->declare_parameter<int>("mpc_contract_version", 2);
  if (!controller_node->has_parameter("mpc_world_frame"))
    controller_node->declare_parameter<std::string>("mpc_world_frame", WORLD_FRAME_NAME);
  if (controller_node->get_parameter("mpc_contract_version").as_int() != 2 ||
      controller_node->get_parameter("mpc_world_frame").as_string() != WORLD_FRAME_NAME)
    throw std::invalid_argument("The MPC transport requires contract version 2 in the world frame");

  // Defaults
  double default_duty_factor = 0.3;
  if (!controller_node->get_parameter("default_duty_factor", default_duty_factor))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_duty_factor given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_duty_factor);
  }

  double default_swing_frequency = 3.0; // [Hz]
  if (!controller_node->get_parameter("default_swing_frequency", default_swing_frequency))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_swing_frequency given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_swing_frequency);
  }

  double default_contact_threshold = 50.0; // [N]
  if (!controller_node->get_parameter("default_contact_threshold", default_contact_threshold))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_contact_threshold given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_contact_threshold);
  }

  double default_step_height = 0.05; // [m]
  if (!controller_node->get_parameter("default_step_height", default_step_height))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_step_height given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_step_height);
  }

  double max_step_height = 0.15; // [m]
  if (!controller_node->get_parameter("max_step_height", max_step_height))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No max_step_height given in namespace %s, using a max value of %f.",
                controller_node->get_namespace(), max_step_height);
  }

  double max_step_length = 0.5; // [m]
  if (!controller_node->get_parameter("max_step_length", max_step_length))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No max_step_length given in namespace %s, using a max value of %f.",
                controller_node->get_namespace(), max_step_length);
  }

  double default_step_reflex_contact_threshold = default_contact_threshold / 3.0; // [N]
  if (!controller_node->get_parameter("default_step_reflex_contact_threshold", default_step_reflex_contact_threshold))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_step_reflex_contact_threshold given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_step_reflex_contact_threshold);
  }

  double default_step_reflex_max_retraction = max_step_height / 2.0; // [m]
  if (!controller_node->get_parameter("default_step_reflex_max_retraction", default_step_reflex_max_retraction))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_step_reflex_max_retraction given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_step_reflex_max_retraction);
  }

  double max_base_height = 0.5; // [m]
  if (!controller_node->get_parameter("max_base_height", max_base_height))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No max_base_height given in namespace %s, using a max value of %f.",
                controller_node->get_namespace(), max_base_height);
  }

  double max_base_roll = 2 * M_PI; // [rad]
  if (!controller_node->get_parameter("max_base_roll", max_base_roll))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No max_base_roll given in namespace %s, using a max value of %f.",
                controller_node->get_namespace(), max_base_roll);
  }

  double max_base_pitch = 2 * M_PI; // [rad]
  if (!controller_node->get_parameter("max_base_pitch", max_base_pitch))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No max_base_pitch given in namespace %s, using a max value of %f.",
                controller_node->get_namespace(), max_base_pitch);
  }

  double min_base_roll = -2 * M_PI; // [rad]
  if (!controller_node->get_parameter("min_base_roll", min_base_roll))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No min_base_roll given in namespace %s, using a min value of %f.",
                controller_node->get_namespace(), min_base_roll);
  }

  double min_base_pitch = -2 * M_PI; // [rad]
  if (!controller_node->get_parameter("min_base_pitch", min_base_pitch))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No min_base_pitch given in namespace %s, using a min value of %f.",
                controller_node->get_namespace(), min_base_pitch);
  }

  double default_base_linear_velocity, default_base_linear_velocity_x, default_base_linear_velocity_y, default_base_linear_velocity_z;
  default_base_linear_velocity = default_base_linear_velocity_x = default_base_linear_velocity_y = default_base_linear_velocity_z = 0.5; // [m/s]
  controller_node->get_parameter("default_base_linear_velocity", default_base_linear_velocity);
  if (controller_node->get_parameter("default_base_linear_velocity", default_base_linear_velocity)) {
    default_base_linear_velocity_x = default_base_linear_velocity_y = default_base_linear_velocity_z = default_base_linear_velocity;
  } else {
    RCLCPP_WARN(controller_node->get_logger(), "No default_base_linear_velocity given in namespace %s, looking for default_base_linear_velocity_[x,y,z].",
                controller_node->get_namespace());
    controller_node->get_parameter("default_base_linear_velocity_x", default_base_linear_velocity_x);
    controller_node->get_parameter("default_base_linear_velocity_y", default_base_linear_velocity_y);
    controller_node->get_parameter("default_base_linear_velocity_z", default_base_linear_velocity_z);
    RCLCPP_WARN(controller_node->get_logger(), "No default_base_linear_velocity_[x,y,z] given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_base_linear_velocity);
  }

  double default_base_angular_velocity, default_base_angular_velocity_roll, default_base_angular_velocity_pitch, default_base_angular_velocity_yaw;
  default_base_angular_velocity = default_base_angular_velocity_roll = default_base_angular_velocity_pitch = default_base_angular_velocity_yaw = 0.5; // [rad/s]
  controller_node->get_parameter("default_base_angular_velocity", default_base_angular_velocity);
  if (controller_node->get_parameter("default_base_angular_velocity", default_base_angular_velocity)) {
    default_base_angular_velocity_roll = default_base_angular_velocity_pitch = default_base_angular_velocity_yaw = default_base_angular_velocity;
  } else {
    RCLCPP_WARN(controller_node->get_logger(), "No default_base_angular_velocity given in namespace %s, looking for default_base_angular_velocity_[roll,pitch,yaw].",
                controller_node->get_namespace());
    controller_node->get_parameter("default_base_angular_velocity_roll", default_base_angular_velocity_roll);
    controller_node->get_parameter("default_base_angular_velocity_pitch", default_base_angular_velocity_pitch);
    controller_node->get_parameter("default_base_angular_velocity_yaw", default_base_angular_velocity_yaw);
    RCLCPP_WARN(controller_node->get_logger(), "No default_base_angular_velocity_[roll,pitch,yaw] given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_base_angular_velocity);
  }

  double default_friction_cones_mu = 0.7;
  if (!controller_node->get_parameter("default_friction_cones_mu", default_friction_cones_mu))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_friction_cones_mu given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_friction_cones_mu);
  }

  double default_cutoff_freq_gyroscope = 300.0; // [Hz]
  if (!controller_node->get_parameter("default_cutoff_freq_gyroscope", default_cutoff_freq_gyroscope))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_cutoff_freq_gyroscope given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_cutoff_freq_gyroscope);
  }

  double default_cutoff_freq_accelerometer = 300.0; // [Hz]
  if (!controller_node->get_parameter("default_cutoff_freq_accelerometer", default_cutoff_freq_accelerometer))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_cutoff_freq_accelerometer given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_cutoff_freq_accelerometer);
  }

  double default_cutoff_freq_qdot = 300.0; // [Hz]
  if (!controller_node->get_parameter("default_cutoff_freq_qdot", default_cutoff_freq_qdot))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_cutoff_freq_qdot given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_cutoff_freq_qdot);
  }

  double default_push_recovery_sensibility = 0.0; // [0.0,1.0]
  if (!controller_node->get_parameter("default_push_recovery_sensibility", default_push_recovery_sensibility))
  {
    RCLCPP_WARN(controller_node->get_logger(), "No default_push_recovery_sensibility given in namespace %s, using a default value of %f.",
                controller_node->get_namespace(), default_push_recovery_sensibility);
  }

  bool activate_com_z = true;
  controller_node->get_parameter("activate_com_z", activate_com_z);
  controller_->getIDProblem()->activateComZ(activate_com_z);

  bool activate_postural = false;
  controller_node->get_parameter("activate_postural", activate_postural);
  controller_->getIDProblem()->activatePostural(activate_postural);

  bool activate_angular_momentum = true;
  controller_node->get_parameter("activate_angular_momentum", activate_angular_momentum);
  controller_->getIDProblem()->activateAngularMomentum(activate_angular_momentum);

  bool activate_joint_position_limits = false;
  controller_node->get_parameter("activate_joint_position_limits", activate_joint_position_limits);
  controller_->getIDProblem()->activateJointPositionLimits(activate_joint_position_limits);

  double regularization = 1e-3;
  controller_node->get_parameter("regularization", regularization);
  controller_->getIDProblem()->setRegularization(regularization);

  std::string qp_solver = "eiQuadProg";
  if (!controller_node->get_parameter("qp_solver", qp_solver))
  {
    RCLCPP_WARN(controller_node->get_logger(),
                "No qp_solver given in namespace %s, using default %s.",
                controller_node->get_namespace(), qp_solver.c_str());
  }
  if(!controller_->getIDProblem()->setSolverByName(qp_solver))
  {
    RCLCPP_WARN(controller_node->get_logger(),
                "Unknown qp_solver '%s'. Keeping default solver.",
                qp_solver.c_str());
  }

  double min_forces_weight = 0.0;
  controller_node->get_parameter("min_forces_weight", min_forces_weight);
  controller_->getIDProblem()->setForcesMinimizationWeight(min_forces_weight);

  double min_qddot_weight = 0.0;
  controller_node->get_parameter("min_qddot_weight", min_qddot_weight);
  controller_->getIDProblem()->setJointAccelerationMinimizationWeight(min_qddot_weight);

  std::string estimation_position_type;
  if (!controller_node->get_parameter("estimation_position_type", estimation_position_type)) {
    RCLCPP_WARN(controller_node->get_logger(), "No estimation_position_type given in namespace %s, using %s",
                controller_node->get_namespace(), controller_->getStateEstimator()->getPositionEstimationType().c_str());
  } else {
    controller_->getStateEstimator()->setPositionEstimationType(estimation_position_type);
  }

  std::string estimation_orientation_type;
  if (!controller_node->get_parameter("estimation_orientation_type", estimation_orientation_type)) {
    RCLCPP_WARN(controller_node->get_logger(), "No estimation_orientation_type given in namespace %s, using %s",
                controller_node->get_namespace(), controller_->getStateEstimator()->getOrientationEstimationType().c_str());
  } else {
    controller_->getStateEstimator()->setOrientationEstimationType(estimation_orientation_type);
  }

  controller_->getStateEstimator()->setContactThreshold(default_contact_threshold);


  // Gait Generator
  controller_->getGaitGenerator()->setSwingFrequency(default_swing_frequency);
  controller_->getGaitGenerator()->setDutyFactor(default_duty_factor);
  controller_->getGaitGenerator()->setStepReflexContactThreshold(default_step_reflex_contact_threshold);
  controller_->getGaitGenerator()->setStepReflexMaxRetraction(default_step_reflex_max_retraction);

  // Footholds planner
  controller_->getFootholdsPlanner()->setBaseLinearVelocityCmd(default_base_linear_velocity_x,default_base_linear_velocity_y,default_base_linear_velocity_z);
  controller_->getFootholdsPlanner()->setBaseAngularVelocityCmd(default_base_angular_velocity_roll,default_base_angular_velocity_pitch,default_base_angular_velocity_yaw);
  controller_->getFootholdsPlanner()->setStepHeight(default_step_height);
  controller_->getFootholdsPlanner()->setMaxStepHeight(max_step_height);
  controller_->getFootholdsPlanner()->setMaxStepLength(max_step_length);
  controller_->getFootholdsPlanner()->setMaxBaseHeight(max_base_height);
  controller_->getFootholdsPlanner()->setMaxBaseRoll(max_base_roll);
  controller_->getFootholdsPlanner()->setMaxBasePitch(max_base_pitch);
  controller_->getFootholdsPlanner()->setMinBaseRoll(min_base_roll);
  controller_->getFootholdsPlanner()->setMinBasePitch(min_base_pitch);
  controller_->getFootholdsPlanner()->setPushRecoverySensibility(default_push_recovery_sensibility);

  // ID problem
  controller_->getIDProblem()->setFrictionConesMu(default_friction_cones_mu);

  // Filters
  controller_->setCutoffFreqQdot(default_cutoff_freq_qdot);
  controller_->setCutoffFreqGyroscope(default_cutoff_freq_gyroscope);
  controller_->setCutoffFreqAccelerometer(default_cutoff_freq_accelerometer);

  // CMD interface
  controller_->setBaseLinearVelocityCmdX(default_base_linear_velocity_x);
  controller_->setBaseLinearVelocityCmdY(default_base_linear_velocity_y);
  controller_->setBaseLinearVelocityCmdZ(default_base_linear_velocity_z);
  controller_->setBaseAngularVelocityCmdRoll(default_base_angular_velocity_roll);
  controller_->setBaseAngularVelocityCmdPitch(default_base_angular_velocity_pitch);
  controller_->setBaseAngularVelocityCmdYaw(default_base_angular_velocity_yaw);

  bool activate_push_recovery = false;
  controller_node->get_parameter("activate_push_recovery", activate_push_recovery);
  controller_->getFootholdsPlanner()->startPushRecovery(activate_push_recovery);

  bool activate_step_reflex = false;
  controller_node->get_parameter("activate_step_reflex", activate_step_reflex);
  controller_->getGaitGenerator()->startStepReflex(activate_step_reflex);

  // Getting Kp and Kd gains
  // Legs
  Eigen::Vector3d Kp_leg, Kd_leg;
  Kp_leg = Kd_leg = Eigen::Vector3d::Ones();
  for (unsigned int i = 0; i < wolf_controller::_joints_prefix.size(); i++) {
    double Kp_leg_i, Kd_leg_i;
    if (!controller_node->get_parameter("gains.Kp_leg." + wolf_controller_utils::_joints_prefix[i], Kp_leg_i)) {
      RCLCPP_WARN(controller_node->get_logger(), "No default Kp_leg_%s gain given in the namespace: %s using 1.0 gain.",
                  wolf_controller::_joints_prefix[i].c_str(), controller_node->get_namespace());
      Kp_leg_i = 1.0;
    }
    if (!controller_node->get_parameter("gains.Kd_leg." + wolf_controller_utils::_joints_prefix[i], Kd_leg_i)) {
      RCLCPP_WARN(controller_node->get_logger(), "No default Kd_leg_%s gain given in the namespace: %s using 1.0 gain.",
                  wolf_controller::_joints_prefix[i].c_str(), controller_node->get_namespace());
      Kd_leg_i = 1.0;
    }
    // Keep ROS1 behavior: invalid gains (missing/NaN/Inf/negative) fall back to 1.0.
    if (!std::isfinite(Kp_leg_i) || !std::isfinite(Kd_leg_i) || Kp_leg_i < 0.0 || Kd_leg_i < 0.0) {
      RCLCPP_WARN(controller_node->get_logger(), "Kp_leg and Kd_leg gains must be finite and non-negative.");
      Kp_leg_i = Kd_leg_i = 1.0;
    }
    Kp_leg(i) = Kp_leg_i;
    Kd_leg(i) = Kd_leg_i;
  }
  controller_ptr->getImpedance()->setLegsGains(Kp_leg, Kd_leg);

  // Arms
  if (controller_->getRobotModel()->getNumberArms() > 0) {
    unsigned int n_joint_arms = controller_->getRobotModel()->getLimbJointsIds(controller_->getRobotModel()->getArmNames()[0]).size();
    if (n_joint_arms > 0) {
      Eigen::VectorXd Kp_arm(n_joint_arms), Kd_arm(n_joint_arms);
      Kp_arm.setOnes();
      Kd_arm.setOnes();
      for (unsigned int i = 0; i < n_joint_arms; i++) {
        double Kp_arm_i, Kd_arm_i;
        if (!controller_node->get_parameter("gains.Kp_arm.j" + std::to_string(i), Kp_arm_i)) {
          RCLCPP_WARN(controller_node->get_logger(), "No default Kp_arm_j%s gain given in the namespace: %s using 1.0 gain.",
                      std::to_string(i).c_str(), controller_node->get_namespace());
          Kp_arm_i = 1.0;
        }
        if (!controller_node->get_parameter("gains.Kd_arm.j" + std::to_string(i), Kd_arm_i)) {
          RCLCPP_WARN(controller_node->get_logger(), "No default Kd_arm_j%s gain given in the namespace: %s using 1.0 gain.",
                      std::to_string(i).c_str(), controller_node->get_namespace());
          Kd_arm_i = 1.0;
        }
        // Keep ROS1 behavior: invalid gains (missing/NaN/Inf/negative) fall back to 1.0.
        if (!std::isfinite(Kp_arm_i) || !std::isfinite(Kd_arm_i) || Kp_arm_i < 0.0 || Kd_arm_i < 0.0) {
          RCLCPP_WARN(controller_node->get_logger(), "Kp_arm and Kd_arm gains must be finite and non-negative.");
          Kp_arm_i = Kd_arm_i = 1.0;
        }
        Kp_arm(i) = Kp_arm_i;
        Kd_arm(i) = Kd_arm_i;
      }
      controller_ptr->getImpedance()->setArmsGains(Kp_arm, Kd_arm);
    }
  }

  // Real-time publishers
  // Contact forces
  unsigned int n_contacts = controller_->getRobotModel()->getContactNames().size();
  auto contact_forces_pub = controller_node->create_publisher<wolf_msgs::msg::ContactForces>("wolf_controller/contact_forces", rclcpp::QoS(4));
  contact_forces_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::ContactForces>>(contact_forces_pub);
  contact_forces_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
  contact_forces_pub_->msg_.name.resize(n_contacts);
  contact_forces_pub_->msg_.contact.resize(n_contacts);
  contact_forces_pub_->msg_.des_contact.resize(n_contacts);
  contact_forces_pub_->msg_.contact_positions.resize(n_contacts);
  contact_forces_pub_->msg_.contact_forces.resize(n_contacts);
  contact_forces_pub_->msg_.des_contact_forces.resize(n_contacts);

  // Foot holds
  unsigned int n_feet = controller_->getRobotModel()->getNumberLegs();
  auto foot_holds_pub = controller_node->create_publisher<wolf_msgs::msg::FootHolds>("wolf_controller/foot_holds", rclcpp::QoS(4));
  foot_holds_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::FootHolds>>(foot_holds_pub);
  foot_holds_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  foot_holds_pub_->msg_.name.resize(n_feet);
  foot_holds_pub_->msg_.desired_foothold.resize(n_feet);
  foot_holds_pub_->msg_.virtual_foothold.resize(n_feet);

  // Terrain estimation
  auto terrain_estimation_pub = controller_node->create_publisher<wolf_msgs::msg::TerrainEstimation>("wolf_controller/terrain_estimation", rclcpp::QoS(4));
  terrain_estimation_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::TerrainEstimation>>(terrain_estimation_pub);
  terrain_estimation_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;

  // Friction cones
  auto friction_cones_pub = controller_node->create_publisher<wolf_msgs::msg::FrictionCones>("wolf_controller/friction_cones", rclcpp::QoS(4));
  friction_cones_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::FrictionCones>>(friction_cones_pub);
  friction_cones_pub_->msg_.header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  friction_cones_pub_->msg_.foot_positions.resize(n_feet);
  friction_cones_pub_->msg_.cone_axis.resize(n_feet);
  friction_cones_pub_->msg_.mus.resize(n_feet);

  // Capture point
  auto capture_point_pub = controller_node->create_publisher<wolf_msgs::msg::CapturePoint>("wolf_controller/capture_point", rclcpp::QoS(4));
  capture_point_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::CapturePoint>>(capture_point_pub);
  capture_point_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
  capture_point_pub_->msg_.support_polygon.points.resize(N_LEGS);

  // Controller state
  auto controller_state_pub = controller_node->create_publisher<wolf_msgs::msg::ControllerState>("wolf_controller/controller_state", rclcpp::QoS(4));
  controller_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::ControllerState>>(controller_state_pub);
  controller_state_pub_->msg_.states = controller_->getStateMachine()->getStatesAsString();
  controller_state_pub_->msg_.current_state = controller_->getStateMachine()->getStateAsString();
  controller_state_pub_->msg_.modes = controller_->getModesAsString();
  controller_state_pub_->msg_.current_mode = controller_->getModeAsString();

  // The centroidal observation has 24 states and 24 inputs. Resolve model
  // indices once, so arm joints and URDF traversal order cannot shift leg data.
  using namespace wolf_controller::mpc_contract;
  auto* model = controller_->getRobotModel();
  const auto string_list = [&controller_node](const std::string& name,
                                             const std::vector<std::string>& fallback) {
    if (!controller_node->has_parameter(name))
      controller_node->declare_parameter<std::vector<std::string>>(name, fallback);
    return controller_node->get_parameter(name).as_string_array();
  };
  try {
    if (!std::isfinite(model->getMass()) || model->getMass() <= 0.0)
      throw std::invalid_argument("MPC requires a positive finite robot mass");
    const auto contact_defaults = controller_node->has_parameter("mpc_contact_names")
        ? std::vector<std::string>{} : canonicalLegOrder(model->getFootNames());
    mpc_contact_names_ = string_list("mpc_contact_names", contact_defaults);
    if (mpc_contact_names_.size() != kContactCount || !uniqueNames(mpc_contact_names_))
      throw std::invalid_argument("mpc_contact_names must contain four distinct LF,LH,RF,RH feet");
    for (const auto& name : mpc_contact_names_)
      if (std::find(model->getFootNames().begin(), model->getFootNames().end(), name) == model->getFootNames().end())
        throw std::invalid_argument("Unknown MPC foot: " + name);

    std::vector<std::string> joint_defaults;
    if (!controller_node->has_parameter("mpc_joint_names")) {
      for (const auto& leg : canonicalLegOrder(model->getLegNames())) {
        for (const auto index : model->getLimbJointsIds(leg)) {
          const auto& names = model->getJointNames();
          const auto name = std::find_if(names.begin(), names.end(), [model, index](const std::string& value) {
            return model->getJointIndex(value) == static_cast<int>(index);
          });
          if (name == names.end())
            throw std::invalid_argument("MPC leg joint index is absent from the model");
          joint_defaults.push_back(*name);
        }
      }
    }
    const auto joint_names = string_list("mpc_joint_names", joint_defaults);
    if (joint_names.size() != kJointCount || !uniqueNames(joint_names))
      throw std::invalid_argument("mpc_joint_names must contain twelve distinct leg joints in planner order");
    std::vector<unsigned int> leg_joint_indices;
    for (const auto& leg : model->getLegNames()) {
      const auto& indices = model->getLimbJointsIds(leg);
      leg_joint_indices.insert(leg_joint_indices.end(), indices.begin(), indices.end());
    }
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const int index = model->getJointIndex(joint_names[i]);
      if (index < FLOATING_BASE_DOFS || index >= static_cast<int>(model->getJointNum()) ||
          std::find(leg_joint_indices.begin(), leg_joint_indices.end(), static_cast<unsigned int>(index)) == leg_joint_indices.end())
        throw std::invalid_argument("Unknown or invalid MPC joint: " + joint_names[i]);
      mpc_joint_indices_[i] = static_cast<std::size_t>(index);
    }
    auto publisher = controller_node->create_publisher<wolf_msgs::msg::MpcObservation>(
        "wolf_controller/mpc_observation", rclcpp::QoS(1));
    mpc_observation_pub_ = std::make_shared<realtime_tools::RealtimePublisher<wolf_msgs::msg::MpcObservation>>(publisher);
    mpc_observation_pub_->msg_.state.value.resize(kStateSize);
    mpc_observation_pub_->msg_.input.value.resize(kInputSize);
  } catch (const std::exception& error) {
    // A robot without the centroidal MPC contract may still use the normal WPG.
    RCLCPP_WARN(controller_node->get_logger(), "MPC observation disabled: %s", error.what());
  }

  if (!controller_node->has_parameter("mpc_contact_reference_timeout"))
    controller_node->declare_parameter<double>("mpc_contact_reference_timeout", 0.25);
  contact_reference_timeout_ = controller_node->get_parameter("mpc_contact_reference_timeout").as_double();
  if (!std::isfinite(contact_reference_timeout_) || contact_reference_timeout_ <= 0.0)
    throw std::invalid_argument("mpc_contact_reference_timeout must be finite and positive");
  mpc_parameters_callback_ = controller_node->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          const auto& name = parameter.get_name();
          if (name == "mpc_contract_version" || name == "mpc_world_frame" ||
              name == "mpc_joint_names" || name == "mpc_contact_names" ||
              name == "mpc_contact_reference_timeout") {
            result.successful = false;
            result.reason = "The MPC contract is fixed while the controller is configured";
            break;
          }
        }
        return result;
      });
  contact_reference_.initRT(ContactReference{});
  const auto feet = model->getFootNames();
  contact_reference_sub_ = controller_node->create_subscription<wolf_msgs::msg::ContactForces>(
      "wolf_controller/reference/contacts", rclcpp::QoS(1),
      [this, feet, logger = controller_node->get_logger()](wolf_msgs::msg::ContactForces::ConstSharedPtr msg) {
        ContactReference reference;
        if (!reorderContacts(feet, msg->name, msg->des_contact, reference.states)) {
          RCLCPP_WARN(logger, "Rejected invalid MPC contact reference");
          return;
        }
        reference.received = std::chrono::steady_clock::now();
        reference.valid = true;
        contact_reference_.writeFromNonRT(reference);
      });
  controller_->getRobotModel()->getJointPosition(tmp_vectorXd_);

  // DDynamic reconfigure
#ifdef DDYNAMIC_RECONFIGURE
  ddr_server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(controller_nh));
  ddr_server_->registerVariable<bool>("stand_up",false,boost::bind(&wolf_controller::ControllerCore::standUp,controller_,_1),"stand up");
  ddr_server_->registerVariable<bool>("activate_push_recovery",controller_->getFootholdsPlanner()->isPushRecoveryActive(),boost::bind(&wolf_controller::FootholdsPlanner::startPushRecovery,controller_->getFootholdsPlanner(),_1),"activate push recovery");
  ddr_server_->registerVariable<bool>("activate_step_reflex",controller_->getGaitGenerator()->isStepReflexActive(),boost::bind(&wolf_controller::GaitGenerator::startStepReflex,controller_->getGaitGenerator(),_1),"activate step reflex");
  ddr_server_->registerVariable<double>("set_duty_factor",default_duty_factor,boost::bind(&wolf_controller::ControllerCore::setDutyFactor,controller_,_1),"set duty factor",0.0,1.0);
  ddr_server_->registerVariable<double>("set_swing_frequency",default_swing_frequency,boost::bind(&wolf_controller::ControllerCore::setSwingFrequency,controller_,_1),"set swing frequency",0.0,6.0);
  ddr_server_->registerVariable<double>("set_linear_vel_x",default_base_linear_velocity_x,boost::bind(&wolf_controller::ControllerCore::setBaseLinearVelocityCmdX,controller_,_1),"set linear velocity x",0.0,1.0);
  ddr_server_->registerVariable<double>("set_linear_vel_y",default_base_linear_velocity_y,boost::bind(&wolf_controller::ControllerCore::setBaseLinearVelocityCmdY,controller_,_1),"set linear velocity y",0.0,1.0);
  ddr_server_->registerVariable<double>("set_linear_vel_z",default_base_linear_velocity_z,boost::bind(&wolf_controller::ControllerCore::setBaseLinearVelocityCmdZ,controller_,_1),"set linear velocity z",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_roll",default_base_angular_velocity_roll,boost::bind(&wolf_controller::ControllerCore::setBaseAngularVelocityCmdRoll,controller_,_1),"set angular velocity roll",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_pitch",default_base_angular_velocity_pitch,boost::bind(&wolf_controller::ControllerCore::setBaseAngularVelocityCmdPitch,controller_,_1),"set angular velocity pitch",0.0,1.0);
  ddr_server_->registerVariable<double>("set_angular_vel_yaw",default_base_angular_velocity_yaw,boost::bind(&wolf_controller::ControllerCore::setBaseAngularVelocityCmdYaw,controller_,_1),"set angular velocity yaw",0.0,1.0);
  ddr_server_->registerVariable<double>("push_recovery_sensibility",controller_->getFootholdsPlanner()->getPushRecoverySensibility(),boost::bind(&wolf_controller::FootholdsPlanner::setPushRecoverySensibility,controller_->getFootholdsPlanner(),_1),"push recovery sensibility",0.0,1.0);
  //ddr_server_->registerVariable<double>("set_linear_vel",default_base_linear_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseLinearVelocityCmd,controller_->getFootholdsPlanner(),_1),"set linear velocity",0.0,1.0);
  //ddr_server_->registerVariable<double>("set_angular_vel",default_base_angular_velocity,boost::bind(&wolf_controller::FootholdsPlanner::setBaseAngularVelocityCmd,controller_->getFootholdsPlanner(),_1),"set angular velocity",0.0,1.0);
  ddr_server_->registerVariable<double>("set_step_height",default_step_height,boost::bind(&wolf_controller::FootholdsPlanner::setStepHeight,controller_->getFootholdsPlanner(),_1),"set step height",0.0,max_step_height);
  ddr_server_->registerVariable<double>("set_contact_threshold",default_contact_threshold,boost::bind(&wolf_controller::StateEstimator::setContactThreshold,controller_->getStateEstimator(),_1),"set contact threshold",0.0,500.0);

  ddr_server_->registerEnumVariable<std::string>("select_gait","TROT",
                                                 boost::bind(&wolf_controller::ControllerCore::selectGait,controller_,_1),
                                                 "select gait", {{"TROT","TROT"},{"CRAWL","CRAWL"}});

  ddr_server_->registerEnumVariable<std::string>("select_control_mode","WPG",
                                                 boost::bind(&wolf_controller::ControllerCore::selectControlMode,controller_,_1),
                                                 "select mode", {{"Walking pattern generator","WPG"},
                                                                 {"External references",      "EXT"}
                                                 });

  ddr_server_->registerVariable<double>("set_mu",controller_->getIDProblem()->getFrictionConesMu(),boost::bind(&wolf_controller::ControllerCore::setFrictionConesMu,controller_,_1),"set the friction cone value mu",0.0,1.0,controller_->getIDProblem()->CLASS_NAME);
  ddr_server_->registerVariable<double>("set_cutoff_freq_qdot",default_cutoff_freq_qdot,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqQdot,controller_,_1),"set cutoff frequency for the joint velocities",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_gyroscope",default_cutoff_freq_gyroscope,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqGyroscope,controller_,_1),"set cutoff frequency for the imu gyroscope",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_accelerometer",default_cutoff_freq_accelerometer,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqAccelerometer,controller_,_1),"set cutoff frequency for the imu accelerometer",0,1000.0);
  ddr_server_->publishServicesTopics();
#endif

  std::string service_ns = "wolf_controller";

  // ROS services
  switch_control_mode_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/switch_control_mode",
        std::bind(&ControllerRosWrapper::switchControlModeCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_control_mode_ = controller_node->create_service<wolf_msgs::srv::String>(
        service_ns+"/set_control_mode",
        std::bind(&ControllerRosWrapper::setControlModeCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  switch_gait_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/switch_gait",
        std::bind(&ControllerRosWrapper::switchGaitCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  switch_posture_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/switch_posture",
        std::bind(&ControllerRosWrapper::switchPostureCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  stand_up_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/stand_up",
        std::bind(&ControllerRosWrapper::standUpCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  stand_down_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/stand_down",
        std::bind(&ControllerRosWrapper::standDownCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  emergency_stop_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/emergency_stop",
        std::bind(&ControllerRosWrapper::emergencyStopCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  reset_base_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/reset_base",
        std::bind(&ControllerRosWrapper::resetBaseCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  decrease_step_height_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/decrease_step_height",
        std::bind(&ControllerRosWrapper::decreaseStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  increase_step_height_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/increase_step_height",
        std::bind(&ControllerRosWrapper::increaseStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_step_height_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        service_ns+"/set_step_height",
        std::bind(&ControllerRosWrapper::setStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  activate_push_recovery_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/activate_push_recovery",
        std::bind(&ControllerRosWrapper::activatePushRecoveryCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  activate_step_reflex_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/activate_step_reflex",
        std::bind(&ControllerRosWrapper::activateStepReflexCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  increase_swing_frequency_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/increase_swing_frequency",
        std::bind(&ControllerRosWrapper::increaseSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  decrease_swing_frequency_ = controller_node->create_service<std_srvs::srv::Trigger>(
        service_ns+"/decrease_swing_frequency",
        std::bind(&ControllerRosWrapper::decreaseSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_swing_frequency_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        service_ns+"/set_swing_frequency",
        std::bind(&ControllerRosWrapper::setSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_duty_factor_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        service_ns+"/set_duty_factor",
        std::bind(&ControllerRosWrapper::setDutyFactorCB, this, std::placeholders::_1, std::placeholders::_2)
        );

}

// Increase Swing Frequency
bool ControllerRosWrapper::increaseSwingFrequencyCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getGaitGenerator()->increaseSwingFrequency();
  return res->success;
}

// Decrease Swing Frequency
bool ControllerRosWrapper::decreaseSwingFrequencyCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getGaitGenerator()->decreaseSwingFrequency();
  return res->success;
}

// Set Swing Frequency
bool ControllerRosWrapper::setSwingFrequencyCB(const std::shared_ptr<wolf_msgs::srv::Float32::Request> req,
                                               std::shared_ptr<wolf_msgs::srv::Float32::Response> res)
{
  if (req->data >= 0) {
    controller_->setSwingFrequency(req->data);
    res->success = true;
  } else {
    res->success = false;
  }
  return res->success;
}

// Set Duty Factor
bool ControllerRosWrapper::setDutyFactorCB(const std::shared_ptr<wolf_msgs::srv::Float32::Request> req,
                                           std::shared_ptr<wolf_msgs::srv::Float32::Response> res)
{
  if (req->data >= 0) {
    controller_->setDutyFactor(req->data);
    res->success = true;
  } else {
    res->success = false;
  }
  return res->success;
}

// Activate Push Recovery
bool ControllerRosWrapper::activatePushRecoveryCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getFootholdsPlanner()->togglePushRecovery();
  return res->success;
}

// Activate Step Reflex
bool ControllerRosWrapper::activateStepReflexCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getGaitGenerator()->toggleStepReflex();
  return res->success;
}

// Set Step Height
bool ControllerRosWrapper::setStepHeightCB(const std::shared_ptr<wolf_msgs::srv::Float32::Request> req,
                                           std::shared_ptr<wolf_msgs::srv::Float32::Response> res)
{
  if (req->data >= 0) {
    controller_->getFootholdsPlanner()->setStepHeight(req->data);
    res->success = true;
  } else {
    res->success = false;
  }
  return res->success;
}

// Increase Step Height
bool ControllerRosWrapper::increaseStepHeightCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getFootholdsPlanner()->increaseStepHeight();
  return res->success;
}

// Decrease Step Height
bool ControllerRosWrapper::decreaseStepHeightCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->getFootholdsPlanner()->decreaseStepHeight();
  return res->success;
}

// Switch Control Mode
bool ControllerRosWrapper::switchControlModeCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  auto mode_request = std::make_shared<wolf_msgs::srv::String::Request>();
  mode_request->data = controller_->getRequestedControlMode() == wolf_controller::ControllerCore::WPG ? "EXT" : "WPG";
  auto mode_response = std::make_shared<wolf_msgs::srv::String::Response>();
  setControlModeCB(mode_request, mode_response);
  res->success = mode_response->success;
  return res->success;
}

// Set Control Mode
bool ControllerRosWrapper::setControlModeCB(const std::shared_ptr<wolf_msgs::srv::String::Request> req,
                      std::shared_ptr<wolf_msgs::srv::String::Response> res)
{
  if (req->data == "MPC" || req->data == "EXT") {
    const auto* reference = contact_reference_.readFromNonRT();
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - reference->received).count();
    const bool guarded = req->data == "MPC" || reference->valid;
    const bool contacts_fresh = !wolf_controller::mpc_contract::contactReferenceExpired(
        true, reference->valid, age, contact_reference_timeout_);
    const bool ready = wolf_controller::mpc_contract::allowMpcStart(
        controller_->getStateMachine()->getCurrentState() == wolf_controller::StateMachine::ACTIVE,
        reference->valid, contact_reference_adopted_.load(), contacts_fresh,
        mpc_observation_pub_ && controller_->getIDProblem()->externalMpcReferencesReady(contact_reference_timeout_));
    if (guarded && !ready) {
      res->success = false;
      return false;
    }
  }
  res->success = controller_->selectControlMode(req->data == "MPC" ? "EXT" : req->data);
  return res->success;
}

// Switch Gait
bool ControllerRosWrapper::switchGaitCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->switchGait();
  return res->success;
}

// Switch Posture
bool ControllerRosWrapper::switchPostureCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->switchPosture();
  return res->success;
}

// Emergency Stop
bool ControllerRosWrapper::emergencyStopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->emergencyStop();
  return res->success;
}

// Reset Base
bool ControllerRosWrapper::resetBaseCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->resetBase();
  unsigned int current_mode = controller_->getControlMode();
  unsigned int current_state = controller_->getStateMachine()->getCurrentState();

  while (current_mode == wolf_controller::ControllerCore::RESET) {
    if (current_state == wolf_controller::StateMachine::ANOMALY) {
      res->success = false;
      break;
    }
    current_mode = controller_->getControlMode();
    current_state = controller_->getStateMachine()->getCurrentState();
    std::this_thread::sleep_for(std::chrono::milliseconds(THREADS_SLEEP_TIME_ms));
  }
  return res->success;
}

// Stand Up
bool ControllerRosWrapper::standUpCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->selectPosture("UP");
  unsigned int current_state = controller_->getStateMachine()->getCurrentState();

  while (current_state != wolf_controller::StateMachine::ACTIVE) {
    if (current_state == wolf_controller::StateMachine::ANOMALY) {
      res->success = false;
      break;
    }
    current_state = controller_->getStateMachine()->getCurrentState();
    std::this_thread::sleep_for(std::chrono::milliseconds(THREADS_SLEEP_TIME_ms));
  }
  return res->success;
}

// Stand Down
bool ControllerRosWrapper::standDownCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  controller_->selectPosture("DOWN");
  unsigned int current_state = controller_->getStateMachine()->getCurrentState();

  while (current_state != wolf_controller::StateMachine::IDLE) {
    if (current_state == wolf_controller::StateMachine::ANOMALY) {
      res->success = false;
      break;
    }
    current_state = controller_->getStateMachine()->getCurrentState();
    std::this_thread::sleep_for(std::chrono::milliseconds(THREADS_SLEEP_TIME_ms));
  }
  return res->success;
}

void ControllerRosWrapper::updateExternalReferences()
{
  const bool active = controller_->getStateMachine()->getCurrentState() == wolf_controller::StateMachine::ACTIVE;
  const bool external = active && controller_->getControlMode() == wolf_controller::ControllerCore::EXT;
  const bool pending_external = active && controller_->getRequestedControlMode() == wolf_controller::ControllerCore::EXT;
  if (!external)
    contact_watchdog_stopped_ = false;
  if (contact_watchdog_stopped_)
    return;
  const auto* reference = contact_reference_.readFromRT();
  if (!reference->valid)
    return;  // Preserve legacy EXT clients that do not provide a contact schedule.
  const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - reference->received).count();
  using wolf_controller::mpc_contract::WatchdogAction;
  const bool contacts_fresh = !wolf_controller::mpc_contract::contactReferenceExpired(
      true, reference->valid, age, contact_reference_timeout_);
  const auto action = wolf_controller::mpc_contract::referenceWatchdogAction(
      external, pending_external, reference->valid, contacts_fresh,
      controller_->getIDProblem()->externalMpcReferencesReady(contact_reference_timeout_));
  if (action != WatchdogAction::None) {
    controller_->selectControlMode("WPG");
    if (action == WatchdogAction::StopActive) {
      contact_watchdog_stopped_ = true;
      // Existing emergency stop enters ANOMALY immediately. Merely requesting
      // WPG can wait forever for all feet to contact after a planner failure.
      controller_->emergencyStop();
    }
    // A pending EXT request can wait for foot contact. Cancel it before the
    // core update if the prepared reference set expires while still in WPG.
    contact_reference_adopted_ = false;
    controller_->getIDProblem()->clearPlannedContactStates();
    return;
  }
  if (age <= contact_reference_timeout_) {
    controller_->getIDProblem()->setPlannedContactStates(reference->states);
    // The service must observe actual controller-thread adoption, not only a
    // valid pending non-RT buffer that the RT try-lock has not consumed yet.
    contact_reference_adopted_ = true;
  } else if (!external) {
    contact_reference_adopted_ = false;
    controller_->getIDProblem()->clearPlannedContactStates();
  }
}

void ControllerRosWrapper::publish(const rclcpp::Time& time, const rclcpp::Duration& period)
{

  if(controller_->getIDProblem())
    controller_->getIDProblem()->publish();


  if(controller_state_pub_.get() && controller_state_pub_->trylock())
  {
    controller_state_pub_->msg_.current_state = controller_->getStateMachine()->getStateAsString();
    controller_state_pub_->msg_.current_mode  = controller_->getModeAsString();
    controller_state_pub_->msg_.header.stamp  = time;
    controller_state_pub_->unlockAndPublish();
  }

  const std::vector<std::string>& foot_names = controller_->getRobotModel()->getFootNames();
  std::string current_foot_names;
  if(foot_holds_pub_.get() && foot_holds_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      current_foot_names = foot_names[i];
      foot_holds_pub_->msg_.name[i] = current_foot_names;
      foot_holds_pub_->msg_.desired_foothold[i].x = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).x();
      foot_holds_pub_->msg_.desired_foothold[i].y = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).y();
      foot_holds_pub_->msg_.desired_foothold[i].z = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).z();
      foot_holds_pub_->msg_.virtual_foothold[i].x = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).x();
      foot_holds_pub_->msg_.virtual_foothold[i].y = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).y();
      foot_holds_pub_->msg_.virtual_foothold[i].z = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).z();
    }
    foot_holds_pub_->msg_.header.stamp = time;
    foot_holds_pub_->unlockAndPublish();
  }

  // Note: des_contact is defined only for the feet at the moment (imagine des_contact as the planned contact state, which
  // for the feet is given by the state machine). For this reason we are using only the feet at the moment
  //const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
  //std::string current_contact_name;
  const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
  const auto& desired_contact_states = controller_->getDesiredContactStates();
  const auto& desired_contact_forces = controller_->getDesiredContactForces();
  std::string current_contact_name;
  if(contact_forces_pub_.get() && contact_forces_pub_->trylock())
  {
    for(unsigned int i=0; i <contact_names.size(); i++)
    {
      current_contact_name = contact_names[i];
      contact_forces_pub_->msg_.name[i] = current_contact_name;
      contact_forces_pub_->msg_.contact[i] = controller_->getStateEstimator()->getContacts().at(current_contact_name);
      contact_forces_pub_->msg_.des_contact[i] = i < desired_contact_states.size() && desired_contact_states[i];
      contact_forces_pub_->msg_.contact_positions[i].x = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(0);
      contact_forces_pub_->msg_.contact_positions[i].y = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(1);
      contact_forces_pub_->msg_.contact_positions[i].z = controller_->getStateEstimator()->getContactPositionInWorld().at(current_contact_name)(2);

      contact_forces_pub_->msg_.contact_forces[i].force.x = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(0);
      contact_forces_pub_->msg_.contact_forces[i].force.y = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(1);
      contact_forces_pub_->msg_.contact_forces[i].force.z = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(2);

      contact_forces_pub_->msg_.des_contact_forces[i].force.x = i < desired_contact_forces.size() ? desired_contact_forces[i](0) : 0.0;
      contact_forces_pub_->msg_.des_contact_forces[i].force.y = i < desired_contact_forces.size() ? desired_contact_forces[i](1) : 0.0;
      contact_forces_pub_->msg_.des_contact_forces[i].force.z = i < desired_contact_forces.size() ? desired_contact_forces[i](2) : 0.0;
    }
    contact_forces_pub_->msg_.header.stamp = time;
    contact_forces_pub_->unlockAndPublish();
  }

  if(terrain_estimation_pub_.get() && terrain_estimation_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      terrain_estimation_pub_->msg_.central_point.x = controller_->getTerrainEstimator()->getTerrainPositionWorld().x();
      terrain_estimation_pub_->msg_.central_point.y = controller_->getTerrainEstimator()->getTerrainPositionWorld().y();
      terrain_estimation_pub_->msg_.central_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();

      terrain_estimation_pub_->msg_.terrain_normal.x = controller_->getTerrainEstimator()->getTerrainNormal().x();
      terrain_estimation_pub_->msg_.terrain_normal.y = controller_->getTerrainEstimator()->getTerrainNormal().y();
      terrain_estimation_pub_->msg_.terrain_normal.z = controller_->getTerrainEstimator()->getTerrainNormal().z();
    }
    terrain_estimation_pub_->msg_.header.stamp = time;
    terrain_estimation_pub_->unlockAndPublish();
  }

  if(friction_cones_pub_.get() && friction_cones_pub_->trylock())
  {
    for(unsigned int i=0; i <foot_names.size(); i++)
    {
      friction_cones_pub_->msg_.foot_positions[i].x = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).x();
      friction_cones_pub_->msg_.foot_positions[i].y = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).y();
      friction_cones_pub_->msg_.foot_positions[i].z = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).z();

      friction_cones_pub_->msg_.cone_axis[i].x = controller_->getTerrainEstimator()->getTerrainNormal().x();
      friction_cones_pub_->msg_.cone_axis[i].y = controller_->getTerrainEstimator()->getTerrainNormal().y();
      friction_cones_pub_->msg_.cone_axis[i].z = controller_->getTerrainEstimator()->getTerrainNormal().z();

      friction_cones_pub_->msg_.mus[i].data = (controller_->getIDProblem() ? controller_->getIDProblem()->getFrictionConesMu() : 1.0);
    }
    friction_cones_pub_->msg_.header.stamp = time;
    friction_cones_pub_->unlockAndPublish();
  }

  if(capture_point_pub_.get() && capture_point_pub_->trylock())
  {
    const std::vector<std::string>& ordered_foot_names = controller_->getFootholdsPlanner()->getPushRecovery()->getOrderedFootNames();

    for(unsigned int i=0; i <N_LEGS; i++)
    {
      capture_point_pub_->msg_.support_polygon.points[i].x = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].x();
      capture_point_pub_->msg_.support_polygon.points[i].y = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].y();
      capture_point_pub_->msg_.support_polygon.points[i].z = controller_->getRobotModel()->getFootPositionInWorld(ordered_foot_names[i]).z();
    }
    capture_point_pub_->msg_.com.x = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().x();
    capture_point_pub_->msg_.com.y = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().y();
    capture_point_pub_->msg_.com.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();
    capture_point_pub_->msg_.capture_point.x = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().x();
    capture_point_pub_->msg_.capture_point.y = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().y();
    capture_point_pub_->msg_.capture_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();
    capture_point_pub_->msg_.header.stamp = time;
    capture_point_pub_->unlockAndPublish();
  }

  // Advance model time even when the realtime publisher is busy. Otherwise
  // dropped publications make the MPC clock run slower than the controller.
  if (std::isfinite(period.seconds()) && period.seconds() >= 0.0)
    mpc_time_ += period.seconds();
  else
    mpc_time_ = 0.0;

  if (mpc_observation_pub_ && mpc_observation_pub_->trylock()) {
    using namespace wolf_controller::mpc_contract;
    auto& message = mpc_observation_pub_->msg_;
    controller_->getRobotModel()->getCentroidalMomentum(tmp_vector6d_);
    controller_->getRobotModel()->getFloatingBasePose(tmp_affine3d_);
    const double mass = controller_->getRobotModel()->getMass();
    for (std::size_t i = 0; i < 6; ++i)
      message.state.value[i] = tmp_vector6d_(i) / mass;
    for (std::size_t i = 0; i < 3; ++i)
      message.state.value[6 + i] = tmp_affine3d_.translation()(i);

    base_rpy_ = wolf_controller_utils::rotToRpy(tmp_affine3d_.linear());
    wolf_controller_utils::unwrap(base_rpy_prev_, base_rpy_);
    message.state.value[9] = base_rpy_.z();
    message.state.value[10] = base_rpy_.y();
    message.state.value[11] = base_rpy_.x();
    base_rpy_prev_ = base_rpy_;
    controller_->getRobotModel()->getJointPosition(tmp_vectorXd_);
    for (std::size_t i = 0; i < kJointCount; ++i)
      message.state.value[12 + i] = tmp_vectorXd_(mpc_joint_indices_[i]);

    std::array<bool, kContactCount> contacts{};
    for (std::size_t i = 0; i < kContactCount; ++i) {
      const auto& force = controller_->getStateEstimator()->getContactForce(mpc_contact_names_[i]);
      for (std::size_t axis = 0; axis < 3; ++axis)
        message.input.value[3 * i + axis] = force(axis);
      contacts[i] = controller_->getStateEstimator()->getContact(mpc_contact_names_[i]);
    }
    controller_->getRobotModel()->getJointVelocity(tmp_vectorXd_);
    for (std::size_t i = 0; i < kJointCount; ++i)
      message.input.value[kForceCount + i] = tmp_vectorXd_(mpc_joint_indices_[i]);
    message.time = mpc_time_;
    message.mode = encodeMode(contacts);
    mpc_observation_pub_->unlockAndPublish();
  }

}
