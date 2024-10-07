/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2019
 * @brief This file contains the constructor, destructor, init, stopping and other facilities for the
 * WoLF controller.
 */

// WoLF
#include <wolf_controller/controller_plugin.h>
#include <wolf_controller/controller_wrapper.h>
#include <wolf_controller/devices/joy.h>
#include <wolf_controller/devices/twist.h>
#include <wolf_controller/devices/keyboard.h>

// WoLF controller utils
#include <wolf_controller_utils/tools.h>

// ROS
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <variant> // For std::variant
#include <optional> // For std::optional

// RT GUI
#ifdef RT_GUI
#include <rt_gui/rt_gui_client.h>
using namespace rt_gui;
#endif

// RT LOGGER
#ifdef RT_LOGGER
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace wolf_controller_utils;

namespace wolf_controller {
using GetParameters = rcl_interfaces::srv::GetParameters;

// Function to get a string parameter from a remote node
std::string get_string_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, const std::string& default_value = "", std::chrono::seconds timeout = std::chrono::seconds(10))
{
    std::string delimiter = "/";
    size_t pos = param_path.find(delimiter);
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;  // Return default value on error
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available!", remote_node_name.c_str());
        return default_value;  // Return default value on error
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->values.empty()) {
            auto param_value = result->values[0];
            if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
                return param_value.string_value;  // Return the found value
            } else {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of type string", param_name.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;  // Return default value if the parameter couldn't be retrieved
}

// Function to get an integer parameter from a remote node
int64_t get_int_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(10))
{
    std::string delimiter = "/";
    size_t pos = param_path.find(delimiter);
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;  // Return default value on error
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available!", remote_node_name.c_str());
        return default_value;  // Return default value on error
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->values.empty()) {
            auto param_value = result->values[0];
            if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                return param_value.integer_value;  // Return the found value
            } else {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of type integer", param_name.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;  // Return default value if the parameter couldn't be retrieved
}

// Function to get a double parameter from a remote node
double get_double_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(10))
{
    std::string delimiter = "/";
    size_t pos = param_path.find(delimiter);
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;  // Return default value on error
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available!", remote_node_name.c_str());
        return default_value;  // Return default value on error
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->values.empty()) {
            auto param_value = result->values[0];
            if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                return param_value.double_value;  // Return the found value
            } else {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of type double", param_name.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;  // Return default value if the parameter couldn't be retrieved
}

// Function to get a boolean parameter from a remote node
bool get_bool_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(10))
{
    std::string delimiter = "/";
    size_t pos = param_path.find(delimiter);
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;  // Return default value on error
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available!", remote_node_name.c_str());
        return default_value;  // Return default value on error
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->values.empty()) {
            auto param_value = result->values[0];
            if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
                return param_value.bool_value;  // Return the found value
            } else {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of type bool", param_name.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;  // Return default value if the parameter couldn't be retrieved
}

WolfController::WolfController()
  :ControllerInterface()
  ,stopping_(false)
  ,publish_odom_tf_(false)
  ,publish_odom_msg_(false)
  ,odom_pub_rate_(250)
{

}

WolfController::~WolfController()
{

}

controller_interface::return_type WolfController::init(const string &controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  prev_time_ = get_node()->get_clock()->now();

  std::string urdf, srdf;
  std::string input_device = "keyboard";
  bool use_contact_sensors = false;

  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<double>("period", period_);
    auto_declare<std::string>("robot_name", robot_name_);
    auto_declare<std::string>("tf_prefix", tf_prefix_);
    //auto_declare<std::string>("/robot_description/description", urdf);
    //auto_declare<std::string>("/robot_description_semantic/description", srdf);
    auto_declare<std::string>("imu_sensor_name", imu_name_);
    auto_declare<std::string>("input_device", input_device);
    auto_declare<bool>("use_contact_sensors", use_contact_sensors);
    auto_declare<bool>("publish_odom_tf", publish_odom_tf_);
    auto_declare<bool>("publish_odom_msg", publish_odom_msg_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  // configure
  period_             = get_node()->get_parameter("period").as_double();
  robot_name_         = get_node()->get_parameter("robot_name").as_string();
  tf_prefix_          = get_node()->get_parameter("tf_prefix").as_string();
  urdf                = get_string_parameter_from_remote_node(get_node(), "robot_description/description");
  srdf                = get_string_parameter_from_remote_node(get_node(), "robot_description_semantic/description");
  imu_name_           = get_node()->get_parameter("imu_sensor_name").as_string();
  input_device        = get_node()->get_parameter("input_device").as_string();
  use_contact_sensors = get_node()->get_parameter("use_contact_sensors").as_bool();
  publish_odom_tf_    = get_node()->get_parameter("publish_odom_tf").as_bool();
  publish_odom_msg_   = get_node()->get_parameter("publish_odom_msg").as_bool();

  // Create the controller core
  controller_ = std::make_shared<ControllerCore>();
  controller_->init(period_,urdf,srdf,robot_name_);

  if(input_device == "ps3")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<Ps3JoyHandler>(get_node(),controller_.get())); // Ps3 joy
  else if(input_device == "xbox")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<XboxJoyHandler>(get_node(),controller_.get())); // Xbox joy
  else if(input_device == "spacemouse")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<SpaceJoyHandler>(get_node(),controller_.get())); // Space joy
  else if(input_device == "keyboard")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<KeyboardHandler>(get_node(),controller_.get())); // Keyboard
  devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<TwistHandler>(get_node(),controller_.get(),"priority_twist")); // Twist
  devices_.addDevice(DevicesHandler::priority_t::LOW,std::make_shared<TwistHandler>(get_node(),controller_.get(),"twist")); // Twist

  ros_wrapper_ = std::make_shared<ControllerRosWrapper>(get_node(),controller_.get());

  // NOTE: This has to be done after ros_wrapper creation because we are loading the params with it
  controller_->getIDProblem()->init(robot_name_,period_);

  // Spawn the odom publisher thread
  odom_publisher_thread_= std::make_shared<std::thread>(&WolfController::odomPublisher,this);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration WolfController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : controller_->getJointNames())
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration WolfController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;

  // Add POSITION, VELOCITY, and EFFORT interfaces for the joints
  for (const auto & joint_name : controller_->getJointNames())
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  // Add IMU interfaces
  conf_names.push_back(imu_name_ + "/orientation.x");
  conf_names.push_back(imu_name_ + "/orientation.y");
  conf_names.push_back(imu_name_ + "/orientation.z");
  conf_names.push_back(imu_name_ + "/orientation.w");
  conf_names.push_back(imu_name_ + "/angular_velocity.x");
  conf_names.push_back(imu_name_ + "/angular_velocity.y");
  conf_names.push_back(imu_name_ + "/angular_velocity.z");
  conf_names.push_back(imu_name_ + "/linear_acceleration.x");
  conf_names.push_back(imu_name_ + "/linear_acceleration.y");
  conf_names.push_back(imu_name_ + "/linear_acceleration.z");

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WolfController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WolfController::on_activate(const rclcpp_lifecycle::State &previous_state)
{

  // register handles for position, velocity, and effort state interfaces
  joint_handles_.reserve(controller_->getJointNames().size());

  for (const auto &joint_name : controller_->getJointNames())
  {
    // Find position interface
    const auto position_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const auto &interface) {
      return interface.get_name() == joint_name &&
          interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });

    if (position_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint position state handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    // Find velocity interface
    const auto velocity_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const auto &interface) {
      return interface.get_name() == joint_name &&
          interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });

    if (velocity_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint velocity state handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    // Find effort interface (for state)
    const auto effort_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const auto &interface) {
      return interface.get_name() == joint_name &&
          interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
    });

    if (effort_state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint effort state handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    // Find effort interface (for command)
    const auto effort_command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const auto &interface) {
      return interface.get_name() == joint_name &&
          interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
    });

    if (effort_command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint effort command handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    // Emplace joint handles with position, velocity, and effort state, and effort command interfaces
    joint_handles_.emplace_back(
          JointHandle{
            std::ref(*position_handle),  // Position
            std::ref(*velocity_handle),  // Velocity
            std::ref(*effort_state_handle), // Effort (State)
            std::ref(*effort_command_handle) // Effort (Command)
          });
  }

  // Register IMU handles
  const std::string imu_name = imu_name_;  // Ensure imu_name_ is set
  const auto orientation_x = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "orientation.x";
  });

  const auto orientation_y = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "orientation.y";
  });

  const auto orientation_z = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "orientation.z";
  });

  const auto orientation_w = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "orientation.w";
  });

  const auto angular_velocity_x = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "angular_velocity.x";
  });

  const auto angular_velocity_y = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "angular_velocity.y";
  });

  const auto angular_velocity_z = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "angular_velocity.z";
  });

  const auto linear_acceleration_x = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "linear_acceleration.x";
  });

  const auto linear_acceleration_y = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "linear_acceleration.y";
  });

  const auto linear_acceleration_z = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(), [&imu_name](const auto & interface) {
    return interface.get_name() == imu_name && interface.get_interface_name() == "linear_acceleration.z";
  });

  if (orientation_x == state_interfaces_.cend() || orientation_y == state_interfaces_.cend() ||
      orientation_z == state_interfaces_.cend() || orientation_w == state_interfaces_.cend() ||
      angular_velocity_x == state_interfaces_.cend() || angular_velocity_y == state_interfaces_.cend() ||
      angular_velocity_z == state_interfaces_.cend() || linear_acceleration_x == state_interfaces_.cend() ||
      linear_acceleration_y == state_interfaces_.cend() || linear_acceleration_z == state_interfaces_.cend())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain IMU state interfaces");
    return CallbackReturn::ERROR;
  }

  imu_handle_ = std::make_unique<IMUHandle>(
        std::ref(*orientation_x), std::ref(*orientation_y), std::ref(*orientation_z), std::ref(*orientation_w),
        std::ref(*angular_velocity_x), std::ref(*angular_velocity_y), std::ref(*angular_velocity_z),
        std::ref(*linear_acceleration_x), std::ref(*linear_acceleration_y), std::ref(*linear_acceleration_z));

  return CallbackReturn::SUCCESS;
}

void WolfController::readJoints()
{
  for (unsigned int i = 0; i < joint_handles_.size(); i++)
  {
    controller_->setJointPosition(i+FLOATING_BASE_DOFS,joint_handles_[i].position_state.get().get_value());
    controller_->setJointVelocity(i+FLOATING_BASE_DOFS,joint_handles_[i].velocity_state.get().get_value());
    controller_->setJointAcceleration(i+FLOATING_BASE_DOFS,0.0); // FIXME
    controller_->setJointEffort(i+FLOATING_BASE_DOFS,joint_handles_[i].effort_state.get().get_value());
  }
}

void WolfController::readImu()
{
  tmp_quat_.w() = imu_handle_->orientation_w.get().get_value();
  tmp_quat_.x() = imu_handle_->orientation_x.get().get_value();
  tmp_quat_.y() = imu_handle_->orientation_y.get().get_value();
  tmp_quat_.z() = imu_handle_->orientation_z.get().get_value();

  tmp_gyro_.x() = imu_handle_->angular_velocity_x.get().get_value();
  tmp_gyro_.y() = imu_handle_->angular_velocity_y.get().get_value();
  tmp_gyro_.z() = imu_handle_->angular_velocity_z.get().get_value();

  tmp_acc_.x() = imu_handle_->linear_acceleration_x.get().get_value();
  tmp_acc_.y() = imu_handle_->linear_acceleration_y.get().get_value();
  tmp_acc_.z() = imu_handle_->linear_acceleration_z.get().get_value();

  controller_->setImuOrientation(tmp_quat_);
  controller_->setImuGyroscope(tmp_gyro_);
  controller_->setImuAccelerometer(tmp_acc_);
}

controller_interface::return_type WolfController::update()
{
  /*auto logger = node_->get_logger();
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }*/

  const rclcpp::Time& time = node_->get_clock()->now();

  rclcpp::Duration period = time - prev_time_;

  // Update input devices
  devices_.writeToOutput(period_);

  // Read the ground truth from the hardware interface if available
  //if(!ground_truth_.getName().empty())
  //  readGroundTruth();
  //
  //// Read the contact sensors from the hardware interface if selected
  //if(use_contact_sensors_)
  //  readContactSensors();

  // Read joint values from the hardware interface
  readJoints();

  // Read IMU values from the hardware interface
  readImu();

  // Update controller core
  controller_->update(period_);

  // Write to the hardware interface
  for (unsigned int i = 0; i < joint_handles_.size(); i++)
    joint_handles_[i].effort_command.get().set_value(controller_->getDesiredJointEfforts()(i+FLOATING_BASE_DOFS));

  // Publish
  ros_wrapper_->publish(time,period);
#ifdef RT_LOGGER
  RtLogger::getLogger().publish(time);
#endif

  prev_time_ = time;

  return controller_interface::return_type::OK;
}

void WolfController::odomPublisher()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Start the odomPublisher");

  // Create the following transformations:
  // odom --> base_footprint --> base --> base_stabilized
  //                   `--> world (available only if using ground truth)
  // odom: represents the floating base pose
  // base_footprint: check here https://www.ros.org/reps/rep-0120.html#base-footprint
  // world: ground truth

  Eigen::Affine3d world_T_base;
  Eigen::Affine3d odom_T_base;
  Eigen::Affine3d odom_T_basefoot;
  Eigen::Affine3d basefoot_T_world;
  Eigen::Affine3d basefoot_T_base;
  Eigen::Affine3d base_T_stabilized;
  Eigen::Vector3d tmp_v;
  double estimated_z;
  Eigen::Matrix3d tmp_R;

  rclcpp::Time t_prev;
  auto br = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::TransformStamped basefoot_T_world_msg;
  geometry_msgs::msg::TransformStamped basefoot_T_base_msg;
  geometry_msgs::msg::TransformStamped odom_T_basefoot_msg;
  geometry_msgs::msg::TransformStamped odom_T_base_msg;
  geometry_msgs::msg::TransformStamped base_T_stabilized_msg;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  if (publish_odom_msg_)
    odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>("odometry/robot", 100);

  rclcpp::Rate publishing_rate(odom_pub_rate_);

  auto state_estimator = controller_->getStateEstimator();
  auto robot_model = controller_->getRobotModel();

  while (!stopping_)
  {
    rclcpp::Time t = get_node()->get_clock()->now();

    double dt = t.seconds() - t_prev.seconds();

    if (dt > 0.0) // Avoid publishing duplicated transforms
    {
      // Get base wrt the ground truth
      world_T_base.translation() = state_estimator->getGroundTruthBasePosition();
      world_T_base.linear() = state_estimator->getGroundTruthBaseOrientation().toRotationMatrix();

      // Get the estimated z of the base
      estimated_z = state_estimator->getEstimatedBaseHeight();

      // Create the tf transform between base_footprint -> world
      tmp_v = world_T_base.translation();
      tmp_v(2) = tmp_v(2) - estimated_z;
      rpyToRotTranspose(0.0, 0.0, robot_model->getBaseYawInWorld(), tmp_R);
      tmp_v = -tmp_R * tmp_v;
      basefoot_T_world.translation() = tmp_v;
      basefoot_T_world.linear() = tmp_R;

      // Set coordinates
      basefoot_T_world_msg = tf2::eigenToTransform(basefoot_T_world);
      // Set transform header
      basefoot_T_world_msg.header.frame_id = tf_prefix_ + BASE_FOOTPRINT_FRAME;
      basefoot_T_world_msg.child_frame_id = tf_prefix_ + WORLD_FRAME_NAME;
      basefoot_T_world_msg.header.stamp = t;
      br->sendTransform(basefoot_T_world_msg);

      // Create the tf transform between base_footprint -> base
      basefoot_T_base.linear() = robot_model->getBaseRotationInHf();
      basefoot_T_base.translation().x() = 0.0;
      basefoot_T_base.translation().y() = 0.0;
      basefoot_T_base.translation().z() = estimated_z;

      // Set coordinates
      basefoot_T_base_msg = tf2::eigenToTransform(basefoot_T_base);
      // Set transform header
      basefoot_T_base_msg.header.frame_id = tf_prefix_ + BASE_FOOTPRINT_FRAME;
      basefoot_T_base_msg.child_frame_id = tf_prefix_ + robot_model->getBaseLinkName();
      basefoot_T_base_msg.header.stamp = t;
      br->sendTransform(basefoot_T_base_msg);

      // Create the tf transform between base -> base_stabilized
      base_T_stabilized.linear() = robot_model->getBaseRotationInHf().transpose();
      base_T_stabilized.translation().x() = 0.0;
      base_T_stabilized.translation().y() = 0.0;
      base_T_stabilized.translation().z() = 0.0;

      // Set coordinates
      base_T_stabilized_msg = tf2::eigenToTransform(base_T_stabilized);
      // Set transform header
      base_T_stabilized_msg.header.frame_id = tf_prefix_ + robot_model->getBaseLinkName();
      base_T_stabilized_msg.child_frame_id = tf_prefix_ + BASE_STABILIZED_FRAME;
      base_T_stabilized_msg.header.stamp = t;
      br->sendTransform(base_T_stabilized_msg);

      if (publish_odom_msg_ || publish_odom_tf_)
      {
        odom_T_base = state_estimator->getFloatingBasePose();
        odom_T_basefoot = odom_T_base * basefoot_T_base.inverse();

        // Set coordinates
        odom_T_basefoot_msg = tf2::eigenToTransform(odom_T_basefoot);
        // Set transform header
        odom_T_basefoot_msg.header.frame_id = tf_prefix_ + ODOM_FRAME;
        odom_T_basefoot_msg.child_frame_id = tf_prefix_ + BASE_FOOTPRINT_FRAME;
        odom_T_basefoot_msg.header.stamp = t;

        if (publish_odom_tf_)
          br->sendTransform(odom_T_basefoot_msg);

        // Create the odom message
        odom_msg.header.stamp = t;
        odom_msg.header.frame_id = odom_T_basefoot_msg.header.frame_id;
        odom_msg.child_frame_id = odom_T_basefoot_msg.child_frame_id;
        odom_msg.pose.pose.position.x = odom_T_basefoot_msg.transform.translation.x;
        odom_msg.pose.pose.position.y = odom_T_basefoot_msg.transform.translation.y;
        odom_msg.pose.pose.position.z = odom_T_basefoot_msg.transform.translation.z;
        odom_msg.pose.pose.orientation = odom_T_basefoot_msg.transform.rotation;
        odom_msg.twist.twist = tf2::toMsg(state_estimator->getFloatingBaseTwist());

        // FIXME This is causing issues:
        // wolf_estimation::eigenToCovariance(odom_estimator.getPoseCovariance(), odom_msg.pose.covariance);
        // wolf_estimation::eigenToCovariance(odom_estimator.getTwistCovariance(), odom_msg.twist.covariance);

        if (publish_odom_msg_)
          odom_pub->publish(odom_msg);
      }
    }

    t_prev = t;

    publishing_rate.sleep();
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Stop the odomPublisher");
}

} //namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(wolf_controller::WolfController, controller_interface::ControllerInterface)
