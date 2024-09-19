/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef CONTROLLER_PLUGIN_H
#define CONTROLLER_PLUGIN_H

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

// PluginLib
#include <pluginlib/class_list_macros.hpp>

// ROS2 control
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
//#include <hardware_interface/imu_sensor_interface.hpp>
//#include <hardware_interface/joint_command_interface.hpp>

// WoLF
#include <wolf_controller_core/controller_core.h>
#include <wolf_controller_ros/devices/interface.h>
#include <wolf_controller_ros/controller_wrapper.h>
//#include <wolf_hardware_interface/ground_truth_interface.h>
//#include <wolf_hardware_interface/contact_switch_sensor_interface.h>
#include <wolf_controller_utils/tools.h>

// Eigen
#include <Eigen/Geometry>

// STD
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>

namespace wolf_controller
{

class Controller : public
    controller_interface::ControllerInterface
{
public:

  const std::string CLASS_NAME = "Controller";

  /**
     * @brief Shared pointer to Controller
     */
  typedef std::shared_ptr<Controller> Ptr;

  /**
     * @brief Weak pointer to Controller
     */
  typedef std::weak_ptr<Controller> WeakPtr;

  /**
     * @brief Shared pointer to const Controller
     */
  typedef std::shared_ptr<const Controller> ConstPtr;

  /** @brief Constructor function */
  Controller();

  /** @brief Destructor function */
  ~Controller() override;

  controller_interface::return_type init(const std::string &controller_name) override;

  controller_interface::return_type update() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const;

  controller_interface::InterfaceConfiguration state_interface_configuration() const;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

private:

  struct JointHandle
  {
    // References to state interfaces (position, velocity, effort)
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> effort_state;

    // Reference to command interface (effort command)
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> effort_command;

    // Constructor to initialize the struct
    JointHandle(
        const hardware_interface::LoanedStateInterface &position,
        const hardware_interface::LoanedStateInterface &velocity,
        const hardware_interface::LoanedStateInterface &effort_state,
        hardware_interface::LoanedCommandInterface &effort_command)
        : position_state(position),
          velocity_state(velocity),
          effort_state(effort_state),
          effort_command(effort_command)
    {}
  };

  struct IMUHandle
  {
    // References to IMU state interfaces (orientation, angular velocity, linear acceleration)
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_x;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_y;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_z;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_w;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_x;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_y;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_z;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_x;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_y;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_z;

    // Constructor to initialize the struct
    IMUHandle(
      const hardware_interface::LoanedStateInterface &orientation_x_,
      const hardware_interface::LoanedStateInterface &orientation_y_,
      const hardware_interface::LoanedStateInterface &orientation_z_,
      const hardware_interface::LoanedStateInterface &orientation_w_,
      const hardware_interface::LoanedStateInterface &angular_velocity_x_,
      const hardware_interface::LoanedStateInterface &angular_velocity_y_,
      const hardware_interface::LoanedStateInterface &angular_velocity_z_,
      const hardware_interface::LoanedStateInterface &linear_acceleration_x_,
      const hardware_interface::LoanedStateInterface &linear_acceleration_y_,
      const hardware_interface::LoanedStateInterface &linear_acceleration_z_)
      : orientation_x(orientation_x_),
        orientation_y(orientation_y_),
        orientation_z(orientation_z_),
        orientation_w(orientation_w_),
        angular_velocity_x(angular_velocity_x_),
        angular_velocity_y(angular_velocity_y_),
        angular_velocity_z(angular_velocity_z_),
        linear_acceleration_x(linear_acceleration_x_),
        linear_acceleration_y(linear_acceleration_y_),
        linear_acceleration_z(linear_acceleration_z_)
    {}
  };

  std::vector<JointHandle> joint_handles_;

  std::unique_ptr<IMUHandle> imu_handle_;

  /** @brief Robot name */
  std::string robot_name_;
  /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
  //std::vector<hardware_interface::LoanedCommandInterface> joint_commands_;
  //std::vector<hardware_interface::LoanedStateInterface> joint_states_;
  /** @brief TF prefix */
  std::string tf_prefix_;
  /** @brief Control period */
  double period_;
  /** @brief Prev time */
  rclcpp::Time prev_time_;
  /** @brief IMU sensor name */
  std::string imu_name_;
  /** @brief IMU sensors */
  //hardware_interface::LoanedStateInterface imu_sensor_;
  /** @brief Ground Truth */
  //hardware_interface::LoanedStateInterface ground_truth_;
  /** @brief Contact sensors */
  //std::map<std::string, hardware_interface::LoanedStateInterface> contact_sensors_;
  /** @brief Ground Truth Orientation */
  //Eigen::Quaterniond ground_truth_orientation_;
  /** @brief Thread for the odometry publisher */
  std::shared_ptr<std::thread> odom_publisher_thread_;
  /** @brief ROS2 node handle */
  //rclcpp::Node::SharedPtr nh_;
  /** @brief Devices Handler */
  DevicesHandler devices_;
  /** @brief Manage the ros interfacing */
  ControllerRosWrapper::Ptr ros_wrapper_;
  /** @brief True if the controller uses the external contact sensors */
  bool use_contact_sensors_;
  /** @brief True if the controller is stopping */
  std::atomic<bool> stopping_;
  /** @brief Publish odom tf */
  bool publish_odom_tf_;
  /** @brief Publish odom msg */
  bool publish_odom_msg_;
  /** @brief Odom publisher rate */
  double odom_pub_rate_;
  /** @brief Controller core */
  ControllerCore::Ptr controller_;
  /** @brief TMP variables */
  Eigen::Quaterniond tmp_quat_;
  Eigen::Vector3d tmp_gyro_;
  Eigen::Vector3d tmp_acc_;

  /**
     * @brief thread body for the odometry publisher
     */
  void odomPublisher();

  /**
     * @brief read the joints state
     */
  void readJoints();

  /**
     * @brief read the imu interface
     */
  void readImu();

  /**
     * @brief read the external estimation with the ground truth interface
     */
  void readGroundTruth();

  /**
     * @brief read the contact sensors with its interface
     */
  void readContactSensors();

};

} // namespace wolf_controller

#endif
