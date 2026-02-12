/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef CONTROLLER_PLUGIN_H
#define CONTROLLER_PLUGIN_H

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

// ROS2 control
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// WoLF
#include <wolf_controller_core/controller_core.h>
#include <wolf_controller/devices/interface.h>
#include <wolf_controller/controller_wrapper.h>
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

class WolfController : public
    controller_interface::ControllerInterface
{
public:

  const std::string CLASS_NAME = "WolfController";

  /**
     * @brief Shared pointer to WolfController
     */
  typedef std::shared_ptr<WolfController> Ptr;

  /**
     * @brief Weak pointer to WolfController
     */
  typedef std::weak_ptr<WolfController> WeakPtr;

  /**
     * @brief Shared pointer to const WolfController
     */
  typedef std::shared_ptr<const WolfController> ConstPtr;

  /** @brief Constructor function */
  WolfController();

  /** @brief Destructor function */
  ~WolfController() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const;

  controller_interface::InterfaceConfiguration state_interface_configuration() const;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

private:

  struct JointHandle
  {
    // References to state interfaces (position, velocity, effort)
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> effort_state_;

    // Reference to command interface (effort command)
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> effort_command_;

    std::string name_;

    // Constructor to initialize the struct
    JointHandle(
        const hardware_interface::LoanedStateInterface &position,
        const hardware_interface::LoanedStateInterface &velocity,
        const hardware_interface::LoanedStateInterface &effort_state,
        hardware_interface::LoanedCommandInterface &effort_command,
        const std::string& name)
        : position_state_(position),
          velocity_state_(velocity),
          effort_state_(effort_state),
          effort_command_(effort_command),
          name_(name)
    {}
  };

  struct IMUHandle
  {
    // References to IMU state interfaces (orientation, angular velocity, linear acceleration)
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_x_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_y_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_z_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> orientation_w_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_x_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_y_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> angular_velocity_z_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_x_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_y_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> linear_acceleration_z_;

    std::string name_;

    // Constructor to initialize the struct
    IMUHandle(
      const hardware_interface::LoanedStateInterface &orientation_x,
      const hardware_interface::LoanedStateInterface &orientation_y,
      const hardware_interface::LoanedStateInterface &orientation_z,
      const hardware_interface::LoanedStateInterface &orientation_w,
      const hardware_interface::LoanedStateInterface &angular_velocity_x,
      const hardware_interface::LoanedStateInterface &angular_velocity_y,
      const hardware_interface::LoanedStateInterface &angular_velocity_z,
      const hardware_interface::LoanedStateInterface &linear_acceleration_x,
      const hardware_interface::LoanedStateInterface &linear_acceleration_y,
      const hardware_interface::LoanedStateInterface &linear_acceleration_z,
      const std::string& name)
      : orientation_x_(orientation_x),
        orientation_y_(orientation_y),
        orientation_z_(orientation_z),
        orientation_w_(orientation_w),
        angular_velocity_x_(angular_velocity_x),
        angular_velocity_y_(angular_velocity_y),
        angular_velocity_z_(angular_velocity_z),
        linear_acceleration_x_(linear_acceleration_x),
        linear_acceleration_y_(linear_acceleration_y),
        linear_acceleration_z_(linear_acceleration_z),
        name_(name)
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
  /** @brief Odom topic */
  std::string odom_topic_;
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
