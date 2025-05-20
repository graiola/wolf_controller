/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef CONTROLLER_WRAPPER_H
#define CONTROLLER_WRAPPER_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/srv/trigger.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wolf_controller_core/common.h>

// Generated
#include <wolf_msgs/msg/contact_forces.hpp>
#include <wolf_msgs/msg/foot_holds.hpp>
#include <wolf_msgs/msg/terrain_estimation.hpp>
#include <wolf_msgs/msg/friction_cones.hpp>
#include <wolf_msgs/msg/capture_point.hpp>
#include <wolf_msgs/msg/controller_state.hpp>
#include <wolf_msgs/srv/float32.hpp>
#include <wolf_msgs/srv/string.hpp>

// WoLF
#include <wolf_controller_core/controller_core.h>

// OCS2
#ifdef OCS2
#include <ocs2_msgs/msg/mpc_observation.hpp>
#endif

// DDYNAMIC RECONFIGURE
#ifdef DDYNAMIC_RECONFIGURE
#include <ddynamic_reconfigure/ddynamic_reconfigure.hpp>
#endif

class ControllerRosWrapper
{

public:

    const std::string CLASS_NAME = "ControllerRosWrapper";

    using Ptr = std::shared_ptr<ControllerRosWrapper>;

    ControllerRosWrapper(rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node, wolf_controller::ControllerCore* const controller_ptr);

    using TriggerRequest = std_srvs::srv::Trigger::Request;
    using TriggerResponse = std_srvs::srv::Trigger::Response;
    using Float32Request = wolf_msgs::srv::Float32::Request;
    using Float32Response = wolf_msgs::srv::Float32::Response;
    using StringRequest = wolf_msgs::srv::String::Request;
    using StringResponse = wolf_msgs::srv::String::Response;

    bool increaseSwingFrequencyCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool decreaseSwingFrequencyCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool setSwingFrequencyCB(const Float32Request::SharedPtr req, Float32Response::SharedPtr res);

    bool setDutyFactorCB(const Float32Request::SharedPtr req, Float32Response::SharedPtr res);

    bool activatePushRecoveryCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool activateStepReflexCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool setStepHeightCB(const Float32Request::SharedPtr req, Float32Response::SharedPtr res);

    bool increaseStepHeightCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool decreaseStepHeightCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool switchControlModeCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool setControlModeCB(const StringRequest::SharedPtr req, StringResponse::SharedPtr res);

    bool switchGaitCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool switchPostureCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool emergencyStopCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool resetBaseCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool standUpCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    bool standDownCB(const TriggerRequest::SharedPtr req, TriggerResponse::SharedPtr res);

    void publish(const rclcpp::Time& time, const rclcpp::Duration& period);

protected:

    /** @brief Real time publisher - controller state */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::ControllerState>> controller_state_pub_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - foot holds */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::FootHolds>> foot_holds_pub_;
    /** @brief Real time publisher - terrain estimation */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::TerrainEstimation>> terrain_estimation_pub_;
    /** @brief Real time publisher - friction cones */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::FrictionCones>> friction_cones_pub_;
    /** @brief Real time publisher - capture point */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::msg::CapturePoint>> capture_point_pub_;
    /** @brief Real time publisher - OCS2 */
    #ifdef OCS2
    std::shared_ptr<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>> mpc_observation_pub_;
    #endif
    /** @brief Controller pnt */
    wolf_controller::ControllerCore* controller_;
    /** @brief ROS services */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_control_mode_;
    rclcpp::Service<wolf_msgs::srv::String>::SharedPtr set_control_mode_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_gait_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_posture_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_down_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_base_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr increase_step_height_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr decrease_step_height_;
    rclcpp::Service<wolf_msgs::srv::Float32>::SharedPtr set_step_height_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activate_push_recovery_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activate_step_reflex_;
    rclcpp::Service<wolf_msgs::srv::Float32>::SharedPtr set_swing_frequency_;
    rclcpp::Service<wolf_msgs::srv::Float32>::SharedPtr set_duty_factor_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr increase_swing_frequency_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr decrease_swing_frequency_;

    /** @brief tmp variables */
    Eigen::Vector6d tmp_vector6d_;
    Eigen::Affine3d tmp_affine3d_;
    Eigen::Vector3d tmp_vector3d_;
    Eigen::VectorXd tmp_vectorXd_;

    /** @brief angles variables */
    Eigen::Vector3d base_rpy_;
    Eigen::Vector3d base_rpy_prev_;

    /** @brief DDynamic reconfigure */
    #ifdef DDYNAMIC_RECONFIGURE
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_server_;
    #endif

};

#endif // CONTROLLER_WRAPPER_H
