/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef DEVICES_ROS_H
#define DEVICES_ROS_H

#include <ros/ros.h>
#include <wolf_controller/devices/interface.h>
#include <wolf_controller_core/common.h>
#include <wolf_controller_core/controller_core.h>
#include <std_msgs/Bool.h>


template <typename msg_t>
class DeviceHandlerRosInterface : public DeviceHandlerInterface
{

public:

    DeviceHandlerRosInterface(ros::NodeHandle& node, wolf_controller::ControllerCore* controller_ptr, const std::string& topic)
        :DeviceHandlerInterface()
    {
        assert(controller_ptr);
        controller_ptr_ = controller_ptr;

        cmd_sub_ = node.subscribe(topic, 1, &DeviceHandlerRosInterface::cmdCallback, this);
    }

    virtual ~DeviceHandlerRosInterface() {}

    virtual void cmdCallback(const msg_t& msg) = 0;

protected:

    void update() // This work as a kind of state machine
    {
        // Clamp the scale values between -1 and 1
        if(base_velocity_x_scale_>1.0) base_velocity_x_scale_ = 1.0; if(base_velocity_x_scale_<-1.0) base_velocity_x_scale_ = -1.0;
        if(base_velocity_y_scale_>1.0) base_velocity_y_scale_ = 1.0; if(base_velocity_y_scale_<-1.0) base_velocity_y_scale_ = -1.0;
        if(base_velocity_z_scale_>1.0) base_velocity_z_scale_ = 1.0; if(base_velocity_z_scale_<-1.0) base_velocity_z_scale_ = -1.0;
        if(base_velocity_roll_scale_>1.0) base_velocity_roll_scale_ = 1.0; if(base_velocity_roll_scale_<-1.0) base_velocity_roll_scale_ = -1.0;
        if(base_velocity_pitch_scale_>1.0) base_velocity_pitch_scale_ = 1.0; if(base_velocity_pitch_scale_<-1.0) base_velocity_pitch_scale_ = -1.0;
        if(base_velocity_yaw_scale_>1.0) base_velocity_yaw_scale_ = 1.0; if(base_velocity_yaw_scale_<-1.0) base_velocity_yaw_scale_ = -1.0;

        unsigned int current_control_mode = controller_ptr_->getControlMode();
        unsigned int current_robot_state = controller_ptr_->getStateMachine()->getCurrentState();

        if(current_robot_state == wolf_controller::StateMachine::ACTIVE)
          if(start_swing_ && current_control_mode == wolf_controller::ControllerCore::WPG)
          {
              controller_ptr_->getFootholdsPlanner()->setCmd(wolf_controller::FootholdsPlanner::LINEAR_AND_ANGULAR); // Start the swing
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleX(base_velocity_x_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleY(base_velocity_y_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleZ(base_velocity_z_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleRoll(base_velocity_roll_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScalePitch(base_velocity_pitch_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleYaw(base_velocity_yaw_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseLinearVelocityCmd(base_velocity_x_cmd_,base_velocity_y_cmd_,base_velocity_z_cmd_);
              controller_ptr_->getFootholdsPlanner()->setBaseAngularVelocityCmd(base_velocity_roll_cmd_,base_velocity_pitch_cmd_,base_velocity_yaw_cmd_);
          }
          else if(std::abs(base_velocity_z_scale_)         >0 ||
                  std::abs(base_velocity_yaw_scale_)       >0 ||
                  std::abs(base_velocity_pitch_scale_)     >0 ||
                  std::abs(base_velocity_roll_scale_)      >0  )
          {
              controller_ptr_->getFootholdsPlanner()->setCmd(wolf_controller::FootholdsPlanner::BASE_ONLY); // Move the base orientation and Z
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleX(0.0);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleY(0.0);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleZ(base_velocity_z_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleYaw(base_velocity_yaw_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScalePitch(base_velocity_pitch_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseVelocityScaleRoll(base_velocity_roll_scale_);
              controller_ptr_->getFootholdsPlanner()->setBaseLinearVelocityCmd(0.0,0.0,base_velocity_z_cmd_);
              controller_ptr_->getFootholdsPlanner()->setBaseAngularVelocityCmd(base_velocity_roll_cmd_,base_velocity_pitch_cmd_,base_velocity_yaw_cmd_);
          }
          else
          {
              controller_ptr_->getFootholdsPlanner()->setCmd(wolf_controller::FootholdsPlanner::HOLD); // HODOR!
          }
    }

    wolf_controller::ControllerCore* controller_ptr_;
    /** @brief Ros subscriber for the device */
    ros::Subscriber cmd_sub_;

};

#endif
