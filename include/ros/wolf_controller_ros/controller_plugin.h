/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef CONTROLLER_PLUGIN_H
#define CONTROLLER_PLUGIN_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// ROS control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
// WoLF
#include <wolf_controller_core/controller_core.h>
#include <wolf_controller_ros/devices/interface.h>
#include <wolf_controller_ros/controller_wrapper.h>
#include <wolf_hardware_interface/ground_truth_interface.h>
#include <wolf_hardware_interface/contact_switch_sensor_interface.h>
#include <wolf_controller_utils/tools.h>

// Eigen
#include <Eigen/Geometry>

namespace wolf_controller
{

class Controller : public
    controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, // Mandatory interface
    hardware_interface::ImuSensorInterface, // Mandatory interface
    hardware_interface::GroundTruthInterface, // Optional interface
    hardware_interface::ContactSwitchSensorInterface> // Optional interface
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
  ~Controller();

  /**
         * @brief Initializes sample controller
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Controller node handle
         */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
  void starting(const ros::Time& time);

  /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
  void stopping(const ros::Time& time);

private:
  /** @brief Robot name */
  std::string robot_name_;
  /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
  std::vector<hardware_interface::JointHandle> joint_states_;
  /** @brief TF prefix */
  std::string tf_prefix_;
  /** @brief Control period */
  double period_;
  /** @brief IMU sensor name */
  std::string imu_name_;
  /** @brief IMU sensors */
  hardware_interface::ImuSensorHandle imu_sensor_;
  /** @brief Ground Thruth */
  hardware_interface::GroundTruthHandle ground_truth_;
  /** @brief Ground Thruth */
  std::map<std::string,hardware_interface::ContactSwitchSensorHandle> contact_sensors_;
  /** @brief Ground Truth Orientation */
  Eigen::Quaterniond ground_truth_orientation_;
  /** @brief Thread for the odometry publisher */
  std::shared_ptr<std::thread> odom_publisher_thread_;
  /** @brief Ros node handle */
  ros::NodeHandle nh_;
  /** @brief Ros node handle */
  ros::NodeHandle root_nh_;
  /** @brief Input devices */
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
  /** @brief TMP variable */
  Eigen::Quaterniond tmp_quat_;

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

} //@namespace wolf_controller

#endif
