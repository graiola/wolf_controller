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
#include <functional>


ControllerRosWrapper::ControllerRosWrapper(rclcpp::Node::SharedPtr controller_node, wolf_controller::ControllerCore* const controller_ptr)
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
    // Check if the values are positive
    if (Kp_leg_i < 0.0 || Kd_leg_i < 0.0) {
      RCLCPP_WARN(controller_node->get_logger(), "Kp_leg and Kd_leg gains must be positive!");
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
        // Check if the values are positive
        if (Kp_arm_i < 0.0 || Kd_arm_i < 0.0) {
          RCLCPP_WARN(controller_node->get_logger(), "Kp_arm and Kd_arm gains must be positive!");
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
  auto contact_forces_pub_ = controller_node->create_publisher<wolf_msgs::msg::ContactForces>("contact_forces", rclcpp::QoS(4));
  auto contact_forces_msg = std::make_shared<wolf_msgs::msg::ContactForces>();
  contact_forces_msg->header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  contact_forces_msg->name.resize(n_contacts);
  contact_forces_msg->contact.resize(n_contacts);
  contact_forces_msg->des_contact.resize(n_contacts);
  contact_forces_msg->contact_positions.resize(n_contacts);
  contact_forces_msg->contact_forces.resize(n_contacts);
  contact_forces_msg->des_contact_forces.resize(n_contacts);

  // Foot holds
  unsigned int n_feet = controller_->getRobotModel()->getNumberLegs();
  auto foot_holds_pub_ = controller_node->create_publisher<wolf_msgs::msg::FootHolds>("foot_holds", rclcpp::QoS(4));
  auto foot_holds_msg = std::make_shared<wolf_msgs::msg::FootHolds>();
  foot_holds_msg->header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  foot_holds_msg->name.resize(n_feet);
  foot_holds_msg->desired_foothold.resize(n_feet);
  foot_holds_msg->virtual_foothold.resize(n_feet);

  // Terrain estimation
  auto terrain_estimation_pub_ = controller_node->create_publisher<wolf_msgs::msg::TerrainEstimation>("terrain_estimation", rclcpp::QoS(4));
  auto terrain_estimation_msg = std::make_shared<wolf_msgs::msg::TerrainEstimation>();
  terrain_estimation_msg->header.frame_id = WORLD_FRAME_NAME;

  // Friction cones
  auto friction_cones_pub_ = controller_node->create_publisher<wolf_msgs::msg::FrictionCones>("friction_cones", rclcpp::QoS(4));
  auto friction_cones_msg = std::make_shared<wolf_msgs::msg::FrictionCones>();
  friction_cones_msg->header.frame_id = controller_ptr->getRobotModel()->getBaseLinkName();
  friction_cones_msg->foot_positions.resize(n_feet);
  friction_cones_msg->cone_axis.resize(n_feet);
  friction_cones_msg->mus.resize(n_feet);

  // Capture point
  auto capture_point_pub_ = controller_node->create_publisher<wolf_msgs::msg::CapturePoint>("capture_point", rclcpp::QoS(4));
  auto capture_point_msg = std::make_shared<wolf_msgs::msg::CapturePoint>();
  capture_point_msg->header.frame_id = WORLD_FRAME_NAME;
  capture_point_msg->support_polygon.points.resize(N_LEGS);

  // Controller state
  auto controller_state_pub_ = controller_node->create_publisher<wolf_msgs::msg::ControllerState>("controller_state", rclcpp::QoS(4));
  auto controller_state_msg = std::make_shared<wolf_msgs::msg::ControllerState>();
  controller_state_msg->states = controller_->getStateMachine()->getStatesAsString();
  controller_state_msg->current_state = controller_->getStateMachine()->getStateAsString();
  controller_state_msg->modes = controller_->getModesAsString();
  controller_state_msg->current_mode = controller_->getModeAsString();

#ifdef OCS2
  // OCS2 MPC observation
  auto mpc_observation_pub_ = controller_node->create_publisher<ocs2_msgs::msg::MpcObservation>("mpc_observation", rclcpp::QoS(4));
  auto mpc_observation_msg = std::make_shared<ocs2_msgs::msg::MpcObservation>();
  mpc_observation_msg->state.value.resize(24);
  mpc_observation_msg->input.value.resize(48);
  mpc_observation_msg->time = 0.0;
  mpc_observation_msg->mode = 0;
  controller_->getRobotModel()->getJointPosition(tmp_vectorXd_);
#endif

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
                                                                 {"Model predictive control", "MPC"},
                                                                 {"External references",      "EXT"}
                                                 });

  ddr_server_->registerVariable<double>("set_mu",controller_->getIDProblem()->getFrictionConesMu(),boost::bind(&wolf_controller::ControllerCore::setFrictionConesMu,controller_,_1),"set the friction cone value mu",0.0,1.0,controller_->getIDProblem()->CLASS_NAME);
  ddr_server_->registerVariable<double>("set_cutoff_freq_qdot",default_cutoff_freq_qdot,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqQdot,controller_,_1),"set cutoff frequency for the joint velocities",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_gyroscope",default_cutoff_freq_gyroscope,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqGyroscope,controller_,_1),"set cutoff frequency for the imu gyroscope",0,1000.0);
  ddr_server_->registerVariable<double>("set_cutoff_freq_accelerometer",default_cutoff_freq_accelerometer,boost::bind(&wolf_controller::ControllerCore::setCutoffFreqAccelerometer,controller_,_1),"set cutoff frequency for the imu accelerometer",0,1000.0);
  ddr_server_->publishServicesTopics();
#endif

  // ROS services
  switch_control_mode_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "switch_control_mode",
        std::bind(&ControllerRosWrapper::switchControlModeCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  switch_gait_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "switch_gait",
        std::bind(&ControllerRosWrapper::switchGaitCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  switch_posture_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "switch_posture",
        std::bind(&ControllerRosWrapper::switchPostureCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  stand_up_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "stand_up",
        std::bind(&ControllerRosWrapper::standUpCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  stand_down_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "stand_down",
        std::bind(&ControllerRosWrapper::standDownCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  emergency_stop_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        std::bind(&ControllerRosWrapper::emergencyStopCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  reset_base_srv_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "reset_base",
        std::bind(&ControllerRosWrapper::resetBaseCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  decrease_step_height_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "decrease_step_height",
        std::bind(&ControllerRosWrapper::decreaseStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  increase_step_height_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "increase_step_height",
        std::bind(&ControllerRosWrapper::increaseStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_step_height_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        "set_step_height",
        std::bind(&ControllerRosWrapper::setStepHeightCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  activate_push_recovery_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "activate_push_recovery",
        std::bind(&ControllerRosWrapper::activatePushRecoveryCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  activate_step_reflex_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "activate_step_reflex",
        std::bind(&ControllerRosWrapper::activateStepReflexCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  increase_swing_frequency_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "increase_swing_frequency",
        std::bind(&ControllerRosWrapper::increaseSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  decrease_swing_frequency_ = controller_node->create_service<std_srvs::srv::Trigger>(
        "decrease_swing_frequency",
        std::bind(&ControllerRosWrapper::decreaseSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_swing_frequency_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        "set_swing_frequency",
        std::bind(&ControllerRosWrapper::setSwingFrequencyCB, this, std::placeholders::_1, std::placeholders::_2)
        );

  set_duty_factor_ = controller_node->create_service<wolf_msgs::srv::Float32>(
        "set_duty_factor",
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
  res->success = true;
  controller_->switchControlMode();
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

void ControllerRosWrapper::publish(const rclcpp::Time& time, const rclcpp::Duration& period)
{

  if(controller_->getIDProblem())
    controller_->getIDProblem()->publish();
/*
  // Controller state publishing
  if(controller_state_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::ControllerState>();
    msg->current_state = controller_->getStateMachine()->getStateAsString();
    msg->current_mode = controller_->getModeAsString();
    msg->header.stamp = time;
    controller_state_pub_->publish(std::move(msg));
  }

  // Footholds publishing
  const std::vector<std::string>& foot_names = controller_->getRobotModel()->getFootNames();
  std::string current_foot_name;

  if(foot_holds_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::FootholdArray>();
    for(unsigned int i = 0; i < foot_names.size(); i++)
    {
      current_foot_name = foot_names[i];
      msg->name[i] = current_foot_name;
      msg->desired_foothold[i].x = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).x();
      msg->desired_foothold[i].y = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).y();
      msg->desired_foothold[i].z = controller_->getFootholdsPlanner()->getDesiredFoothold(foot_names[i]).z();
      msg->virtual_foothold[i].x = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).x();
      msg->virtual_foothold[i].y = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).y();
      msg->virtual_foothold[i].z = controller_->getFootholdsPlanner()->getVirtualFoothold(foot_names[i]).z();
    }
    msg->header.stamp = time;
    foot_holds_pub_->publish(std::move(msg));
  }

  // Contact forces publishing
  const std::vector<std::string>& contact_names = controller_->getRobotModel()->getContactNames();
  std::string current_contact_name;

  if(contact_forces_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::ContactForceArray>();
    for(unsigned int i = 0; i < contact_names.size(); i++)
    {
      current_contact_name = contact_names[i];
      msg->name[i] = current_contact_name;
      msg->contact[i] = controller_->getStateEstimator()->getContacts().at(current_contact_name);
      msg->des_contact[i] = controller_->getDesiredContactStates()[i];
      msg->contact_positions[i].x = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(0);
      msg->contact_positions[i].y = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(1);
      msg->contact_positions[i].z = controller_->getStateEstimator()->getContactPositionInBase().at(current_contact_name)(2);

      msg->contact_forces[i].force.x = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(0);
      msg->contact_forces[i].force.y = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(1);
      msg->contact_forces[i].force.z = controller_->getStateEstimator()->getContactForces().at(current_contact_name)(2);

      msg->des_contact_forces[i].force.x = controller_->getDesiredContactForces()[i](0);
      msg->des_contact_forces[i].force.y = controller_->getDesiredContactForces()[i](1);
      msg->des_contact_forces[i].force.z = controller_->getDesiredContactForces()[i](2);
    }
    msg->header.stamp = time;
    contact_forces_pub_->publish(std::move(msg));
  }

  // Terrain estimation publishing
  if(terrain_estimation_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::TerrainEstimation>();
    msg->central_point.x = controller_->getTerrainEstimator()->getTerrainPositionWorld().x();
    msg->central_point.y = controller_->getTerrainEstimator()->getTerrainPositionWorld().y();
    msg->central_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();

    msg->terrain_normal.x = controller_->getTerrainEstimator()->getTerrainNormal().x();
    msg->terrain_normal.y = controller_->getTerrainEstimator()->getTerrainNormal().y();
    msg->terrain_normal.z = controller_->getTerrainEstimator()->getTerrainNormal().z();

    msg->header.stamp = time;
    terrain_estimation_pub_->publish(std::move(msg));
  }

  // Friction cones publishing
  if(friction_cones_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::FrictionConeArray>();
    for(unsigned int i = 0; i < foot_names.size(); i++)
    {
      msg->foot_positions[i].x = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).x();
      msg->foot_positions[i].y = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).y();
      msg->foot_positions[i].z = controller_->getRobotModel()->getFeetPositionInBase().at(foot_names[i]).z();

      msg->cone_axis[i].x = controller_->getTerrainEstimator()->getTerrainNormal().x();
      msg->cone_axis[i].y = controller_->getTerrainEstimator()->getTerrainNormal().y();
      msg->cone_axis[i].z = controller_->getTerrainEstimator()->getTerrainNormal().z();

      msg->mus[i].data = (controller_->getIDProblem() ? controller_->getIDProblem()->getFrictionConesMu() : 1.0);
    }
    msg->header.stamp = time;
    friction_cones_pub_->publish(std::move(msg));
  }

  // Capture point publishing
  if(capture_point_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::CapturePoint>();
    const std::vector<std::string>& ordered_foot_names = controller_->getFootholdsPlanner()->getPushRecovery()->getOrderedFootNames();

    for(unsigned int i = 0; i < N_LEGS; i++)
    {
      msg->support_polygon.points[i].x = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].x();
      msg->support_polygon.points[i].y = controller_->getFootholdsPlanner()->getPushRecovery()->getSupportPolygonEdges()[i].y();
      msg->support_polygon.points[i].z = controller_->getRobotModel()->getFootPositionInWorld(ordered_foot_names[i]).z();
    }
    msg->com.x = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().x();
    msg->com.y = controller_->getFootholdsPlanner()->getPushRecovery()->getComPositionXY().y();
    msg->com.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();
    msg->capture_point.x = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().x();
    msg->capture_point.y = controller_->getFootholdsPlanner()->getPushRecovery()->getCapturePoint().y();
    msg->capture_point.z = controller_->getTerrainEstimator()->getTerrainPositionWorld().z();

    msg->header.stamp = time;
    capture_point_pub_->publish(std::move(msg));
  }

#ifdef OCS2
  // MPC observation publishing
  if(mpc_observation_pub_)
  {
    auto msg = std::make_unique<wolf_msgs::msg::MpcObservation>();

    controller_->getRobotModel()->getCentroidalMomentum(tmp_vector6d_);
    controller_->getRobotModel()->getFloatingBasePose(tmp_affine3d_);
    double robot_mass = controller_->getRobotModel()->getMass();

    // State - linear momentum, angular momentum, base position, base orientation, joint positions
    msg->state.value[0] = tmp_vector6d_(0) / robot_mass;
    msg->state.value[1] = tmp_vector6d_(1) / robot_mass;
    msg->state.value[2] = tmp_vector6d_(2) / robot_mass;
    msg->state.value[3] = tmp_vector6d_(3) / robot_mass;
    msg->state.value[4] = tmp_vector6d_(4) / robot_mass;
    msg->state.value[5] = tmp_vector6d_(5) / robot_mass;
    msg->state.value[6] = tmp_affine3d_.translation().x();
    msg->state.value[7] = tmp_affine3d_.translation().y();
    msg->state.value[8] = tmp_affine3d_.translation().z();

    base_rpy_ = wolf_controller_utils::rotToRpy(tmp_affine3d_.linear());
    wolf_controller_utils::unwrap(base_rpy_prev_, base_rpy_);
    msg->state.value[9] = base_rpy_.z();
    msg->state.value[10] = base_rpy_.y();
    msg->state.value[11] = base_rpy_.x();

    const Eigen::VectorXd& q_joints = controller_->getRobotModel()->getJointPositions();
    std::copy(q_joints.data(), q_joints.data() + q_joints.size(), &msg->state.value[12]);

    // Input - joint velocities
    const Eigen::VectorXd& dq_joints = controller_->getRobotModel()->getJointVelocities();
    std::copy(dq_joints.data(), dq_joints.data() + dq_joints.size(), &msg->input.value[0]);

    msg->header.stamp = time;
    mpc_observation_pub_->publish(std::move(msg));
  }
#endif
*/
}


