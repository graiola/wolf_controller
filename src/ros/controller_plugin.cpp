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

// RT LOGGER
#ifdef RT_LOGGER
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace wolf_controller_utils;

namespace wolf_controller {

WolfController::WolfController()
  :MultiInterfaceController<hardware_interface::EffortJointInterface,
   hardware_interface::ImuSensorInterface,
   hardware_interface::GroundTruthInterface,
   hardware_interface::ContactSwitchSensorInterface> (true) // allow_optional_interfaces = true
  ,stopping_(false)
  ,publish_odom_tf_(false)
  ,publish_odom_msg_(false)
  ,odom_pub_rate_(250)
{

}

WolfController::~WolfController()
{
}

bool WolfController::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
  ROS_INFO_NAMED(CLASS_NAME,"Initialize controller plugin");

  if(!robot_hw)
  {
    ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::RobotHW is a null pointer!");
    return false;
  }

  nh_ = controller_nh; // /robot_name/wolf_controller
  root_nh_ = root_nh; // /robot_name/


  if(!nh_.getParam("period",period_)) // Get the initial controller period
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No period given in namespace "+nh_.getNamespace());
    return false;
  }

  assert(period_ > 0.0);

  if(!root_nh_.getParam("robot_name",robot_name_)) // Get the robot namespace
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot name given in namespace "+root_nh_.getNamespace());
    return false;
  }

  if(!root_nh_.getParam("tf_prefix",tf_prefix_)) // Get the tf prefix
  {
    ROS_WARN_STREAM_NAMED(CLASS_NAME,"No tf prefix given in namespace, using an empty one "+root_nh_.getNamespace());
  }

  fixTFprefix(tf_prefix_);

  // Create the quadruped robot object, it wraps the robot model with some meta information
  std::string urdf, srdf;
  if(!root_nh_.getParam("robot_description",urdf)) // Get the robot description from the global namespace "/"
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot_description given in namespace " + root_nh_.getNamespace());
    return false;
  }
  if(!root_nh_.getParam("robot_description_semantic",srdf)) // Get the robot semantic description from the global namespace "/"
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No robot_description_semantic given in namespace " + root_nh_.getNamespace());
    return false;
  }

  // Create the controller core
  controller_ = std::make_shared<ControllerCore>();
  controller_->init(period_,urdf,srdf,robot_name_);
  auto joint_names = controller_->getJointNames();

  // Load hardware interfaces
  hardware_interface::EffortJointInterface* jt_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
  hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
  hardware_interface::GroundTruthInterface* gt_hw = robot_hw->get<hardware_interface::GroundTruthInterface>();
  hardware_interface::ContactSwitchSensorInterface* cs_hw = robot_hw->get<hardware_interface::ContactSwitchSensorInterface>();

  // Hardware interfaces: Joints
  if(!jt_hw)
  {
    ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::EffortJointInterface not found");
    return false;
  }
  else
  {
    // Setting up joint handles:
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      // Getting joint state handle
      try
      {
        ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Found joint: "<<joint_names[i]);
        joint_states_.push_back(jt_hw->getHandle(joint_names[i])); // FIXME
      }
      catch(...)
      {
        ROS_ERROR_NAMED(CLASS_NAME,"Error loading the joint handles");
        return false;
      }
    }
    assert(joint_states_.size()>0);
  }

  if(!imu_hw)
  {
    ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ImuSensorInterface not found");
    return false;
  }
  else
  {
    try
    {
      std::string imu_sensor_name;
      if(controller_nh.getParam("imu_sensor_name",imu_sensor_name))
        imu_sensor_ = imu_hw->getHandle(imu_sensor_name); // Take the selected imu sensor
      else
        imu_sensor_ = imu_hw->getHandle(imu_hw->getNames()[0]); // Take the first imu sensor
    }
    catch(...)
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Error loading the imu handler");
      return false;
    }
  }

  if(!gt_hw)
    ROS_WARN_NAMED(CLASS_NAME,"hardware_interface::GroundTruthInterface not found");
  else
    ground_truth_ = gt_hw->getHandle(gt_hw->getNames()[0]);

  use_contact_sensors_ = false;
  controller_nh.getParam("use_contact_sensors",use_contact_sensors_);
  if(use_contact_sensors_)
  {
    if(!cs_hw)
    {
      ROS_ERROR_NAMED(CLASS_NAME,"hardware_interface::ContactSwitchSensorInterface not found");
      return false;
    }
    else
    {
      auto contact_sensor_names = cs_hw->getNames();
      if(contact_sensor_names.size() != N_LEGS)
      {
        ROS_ERROR_NAMED(CLASS_NAME,"Wrong number of contact sensors! only (4) are supported. Did you specify the contacts in the SRDF file?");
        return false;
      }
      auto foot_names = controller_->getRobotModel()->getFootNames();
      contact_sensor_names = sortByLegPrefix(contact_sensor_names);
      for(unsigned int i=0;i<contact_sensor_names.size();i++)
        contact_sensors_[foot_names[i]] = cs_hw->getHandle(contact_sensor_names[i]);
    }
  }
  if(!use_contact_sensors_) // Use the contact sensors
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact estimation");
  else
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Using contact sensors");

  std::string input_device = "keyboard";
  nh_.getParam("input_device",input_device);
  if(input_device == "ps3")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<Ps3JoyHandler>(controller_nh,controller_.get())); // Ps3 joy
  else if(input_device == "xbox")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<XboxJoyHandler>(controller_nh,controller_.get())); // Xbox joy
  else if(input_device == "spacemouse")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<SpaceJoyHandler>(controller_nh,controller_.get())); // Space joy
  else if(input_device == "keyboard")
    devices_.addDevice(DevicesHandler::priority_t::MEDIUM,std::make_shared<KeyboardHandler>(controller_nh,controller_.get())); // Keyboard
  devices_.addDevice(DevicesHandler::priority_t::HIGH,std::make_shared<TwistHandler>(controller_nh,controller_.get(),"priority_twist")); // Twist
  devices_.addDevice(DevicesHandler::priority_t::LOW,std::make_shared<TwistHandler>(controller_nh,controller_.get(),"twist")); // Twist

  bool publish_odom_tf = false; // On/Off
  controller_nh.getParam("publish_odom_tf", publish_odom_tf);
  bool publish_odom_msg = false; // On/Off
  controller_nh.getParam("publish_odom_msg", publish_odom_msg);

  publish_odom_tf_ = publish_odom_tf;
  publish_odom_msg_ = publish_odom_msg;

  ros_wrapper_ = std::make_shared<ControllerRosWrapper>(root_nh,controller_nh,controller_.get());

  // NOTE: This has to be done after ros_wrapper creation because we are loading the params with it
  controller_->getIDProblem()->init(robot_name_,period_);

  // Spawn the odom publisher thread
  odom_publisher_thread_= std::make_shared<std::thread>(&WolfController::odomPublisher,this);

  return true;
}

void WolfController::readJoints()
{
  for (unsigned int i = 0; i < joint_states_.size(); i++)
  {
    controller_->setJointPosition(i+FLOATING_BASE_DOFS,joint_states_[i].getPosition());
    controller_->setJointVelocity(i+FLOATING_BASE_DOFS,joint_states_[i].getVelocity());
    controller_->setJointAcceleration(i+FLOATING_BASE_DOFS,0.0); // FIXME
    controller_->setJointEffort(i+FLOATING_BASE_DOFS,joint_states_[i].getEffort());
  }
}

void WolfController::readImu()
{
  //imu_accelerometer_ = Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getLinearAcceleration());
  //imu_gyroscope_     = Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getAngularVelocity());

  tmp_quat_.w() = imu_sensor_.getOrientation()[0];
  tmp_quat_.x() = imu_sensor_.getOrientation()[1];
  tmp_quat_.y() = imu_sensor_.getOrientation()[2];
  tmp_quat_.z() = imu_sensor_.getOrientation()[3];

  controller_->setImuOrientation(tmp_quat_);
  controller_->setImuGyroscope(Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getAngularVelocity()));
  controller_->setImuAccelerometer(Eigen::Map<const Eigen::Vector3d>(imu_sensor_.getLinearAcceleration()));
}

void WolfController::readGroundTruth()
{

  tmp_quat_.w() = ground_truth_.getOrientation()[0];
  tmp_quat_.x() = ground_truth_.getOrientation()[1];
  tmp_quat_.y() = ground_truth_.getOrientation()[2];
  tmp_quat_.z() = ground_truth_.getOrientation()[3];

  controller_->setExtEstimatedState(Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearPosition()),
                                    Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearVelocity()),
                                    Eigen::Map<const Eigen::Vector3d>(ground_truth_.getLinearAcceleration()),
                                    tmp_quat_,
                                    Eigen::Map<const Eigen::Vector3d>(ground_truth_.getAngularVelocity()));
}

void WolfController::readContactSensors()
{
  for(const auto& tmp : contact_sensors_)
      controller_->setExtEstimatedContactState(tmp.first,tmp.second.getContactState(),Eigen::Map<const Eigen::Vector3d>(tmp.second.getForce()));
}

void WolfController::starting(const ros::Time&  /*time*/)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"Starting WoLF controller");

  // Read from the hardware interfaces:
  // 1) Joints
  readJoints();
  // 2) IMU
  readImu();

  ROS_DEBUG_NAMED(CLASS_NAME,"Starting WoLF controller completed");
}

void WolfController::update(const ros::Time& time, const ros::Duration& period)
{
  period_ = period.toSec();

  // Update input devices
  devices_.writeToOutput(period_);

  // Read the ground truth from the hardware interface if available
  if(!ground_truth_.getName().empty())
    readGroundTruth();

  // Read the contact sensors from the hardware interface if selected
  if(use_contact_sensors_)
    readContactSensors();

  // Read joint values from the hardware interface
  readJoints();

  // Read IMU values from the hardware interface
  readImu();

  // Update controller core
  controller_->update(period_);

  // Write to the hardware interface
  for (unsigned int i = 0; i < joint_states_.size(); i++)
    joint_states_[i].setCommand(controller_->getDesiredJointEfforts()(i+FLOATING_BASE_DOFS));

  // Publish
  ros_wrapper_->publish(time,period);
#ifdef RT_LOGGER
  RtLogger::getLogger().publish(time);
#endif
}

void WolfController::odomPublisher()
{
  ROS_DEBUG_NAMED(CLASS_NAME,"Start the odomPublisher");

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

  ros::Time t_prev;
  tf2_ros::TransformBroadcaster br;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped basefoot_T_world_msg;
  geometry_msgs::TransformStamped basefoot_T_base_msg;
  geometry_msgs::TransformStamped odom_T_basefoot_msg;
  geometry_msgs::TransformStamped odom_T_base_msg;
  geometry_msgs::TransformStamped base_T_stabilized_msg;
  ros::Publisher odom_pub;

  if(publish_odom_msg_)
    odom_pub = root_nh_.advertise<nav_msgs::Odometry>("odometry/robot",100);

  ros::Rate publishing_rate(odom_pub_rate_);

  auto state_estimator = controller_->getStateEstimator();
  auto robot_model = controller_->getRobotModel();

  while(!stopping_)
  {
    ros::Time t = ros::Time::now();

    double dt = t.toSec() - t_prev.toSec();

    if(dt > 0.0) // Avoid publishing duplicated transforms
    {
      // Get base wrt the ground truth
      world_T_base.translation() = state_estimator->getGroundTruthBasePosition();
      world_T_base.linear() = state_estimator->getGroundTruthBaseOrientation().toRotationMatrix();

      // Get the estimated z of the base
      estimated_z = state_estimator->getEstimatedBaseHeight();

      // Create the tf transform between base_footprint -> world
      tmp_v =  world_T_base.translation();
      tmp_v(2) = tmp_v(2) - estimated_z;
      rpyToRotTranspose(0.0,0.0,robot_model->getBaseYawInWorld(),tmp_R);
      tmp_v = - tmp_R * tmp_v;
      basefoot_T_world.translation() = tmp_v;
      basefoot_T_world.linear() = tmp_R;
      // Set coordinates
      basefoot_T_world_msg = tf2::eigenToTransform(basefoot_T_world);
      // Set transform header
      basefoot_T_world_msg.header.frame_id = tf_prefix_+BASE_FOOTPRINT_FRAME;
      basefoot_T_world_msg.child_frame_id  = tf_prefix_+WORLD_FRAME_NAME;
      basefoot_T_world_msg.header.seq++;
      basefoot_T_world_msg.header.stamp = t;
      br.sendTransform(basefoot_T_world_msg);

      // Create the tf transform between base_footprint -> base
      basefoot_T_base.linear() = robot_model->getBaseRotationInHf();
      basefoot_T_base.translation().x() = 0.0;
      basefoot_T_base.translation().y() = 0.0;
      basefoot_T_base.translation().z() = estimated_z;
      // Set coordinates
      basefoot_T_base_msg = tf2::eigenToTransform(basefoot_T_base);
      // Set transform header
      basefoot_T_base_msg.header.frame_id = tf_prefix_+BASE_FOOTPRINT_FRAME;
      basefoot_T_base_msg.child_frame_id  = tf_prefix_+robot_model->getBaseLinkName();
      basefoot_T_base_msg.header.seq++;
      basefoot_T_base_msg.header.stamp = t;
      br.sendTransform(basefoot_T_base_msg);

      // Create the tf transform between base -> base_stabilized
      base_T_stabilized.linear() = robot_model->getBaseRotationInHf().transpose();
      base_T_stabilized.translation().x() = 0.0;
      base_T_stabilized.translation().y() = 0.0;
      base_T_stabilized.translation().z() = 0.0;
      // Set coordinates
      base_T_stabilized_msg = tf2::eigenToTransform(base_T_stabilized);
      // Set transform header
      base_T_stabilized_msg.header.frame_id = tf_prefix_+robot_model->getBaseLinkName();
      base_T_stabilized_msg.child_frame_id  = tf_prefix_+BASE_STABILIZED_FRAME;
      base_T_stabilized_msg.header.seq++;
      base_T_stabilized_msg.header.stamp = t;
      br.sendTransform(base_T_stabilized_msg);

      if(publish_odom_msg_ || publish_odom_tf_)
      {

        odom_T_base = state_estimator->getFloatingBasePose();

        odom_T_basefoot = odom_T_base * basefoot_T_base.inverse();

        // Set coordinates
        odom_T_basefoot_msg = tf2::eigenToTransform(odom_T_basefoot);
        // Set transform header
        odom_T_basefoot_msg.header.frame_id = tf_prefix_+ODOM_FRAME;
        odom_T_basefoot_msg.child_frame_id  = tf_prefix_+BASE_FOOTPRINT_FRAME;
        odom_T_basefoot_msg.header.seq++;
        odom_T_basefoot_msg.header.stamp = t;
        if(publish_odom_tf_)
          br.sendTransform(odom_T_basefoot_msg);

        // Create the odom message
        odom_msg.header.seq                 ++;
        odom_msg.header.stamp               = t;
        odom_msg.header.frame_id            = odom_T_basefoot_msg.header.frame_id;
        odom_msg.child_frame_id             = odom_T_basefoot_msg.child_frame_id;
        odom_msg.pose.pose.position.x       = odom_T_basefoot_msg.transform.translation.x;
        odom_msg.pose.pose.position.y       = odom_T_basefoot_msg.transform.translation.y;
        odom_msg.pose.pose.position.z       = odom_T_basefoot_msg.transform.translation.z;
        odom_msg.pose.pose.orientation      = odom_T_basefoot_msg.transform.rotation;
        odom_msg.twist.twist                = tf2::toMsg(state_estimator->getFloatingBaseTwist());
        // FIXME This is causing issues:
        //wolf_estimation::eigenToCovariance(odom_estimator.getPoseCovariance(),odom_msg.pose.covariance);
        //wolf_estimation::eigenToCovariance(odom_estimator.getTwistCovariance(),odom_msg.twist.covariance);
        if(publish_odom_msg_)
          odom_pub.publish(odom_msg);
      }
    }

    //std::this_thread::sleep_for( std::chrono::milliseconds(THREADS_SLEEP_TIME_ms) );

    t_prev = t;

    publishing_rate.sleep();
  }
  ROS_DEBUG_NAMED(CLASS_NAME,"Stop the odomPublisher");
}

void WolfController::stopping(const ros::Time& /*time*/)
{
  ROS_DEBUG_NAMED(CLASS_NAME,"Stopping WoLF controller");

  stopping_ = true;
  odom_publisher_thread_->join();

  ROS_DEBUG_NAMED(CLASS_NAME,"Stopping WoLF controller completed");
}

} //namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(wolf_controller::WolfController, controller_interface::ControllerBase);
