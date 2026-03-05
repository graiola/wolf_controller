/**
 * @file plot_node.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief plot node.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <wolf_msgs/msg/contact_forces.hpp>
#include <wolf_msgs/msg/capture_point.hpp>
#include <wolf_msgs/msg/cartesian_task.hpp>
#include <wolf_msgs/msg/com_task.hpp>
#include <wolf_msgs/msg/friction_cones.hpp>
#include <wolf_msgs/msg/terrain_estimation.hpp>
#include <wolf_msgs/msg/foot_holds.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <builtin_interfaces/msg/time.hpp>

namespace rviz_visual_tools {

class VisualTools : public RvizVisualTools
{

protected:

  rclcpp::Node::SharedPtr node_;

public:
  using Ptr = std::shared_ptr<VisualTools>;

  VisualTools(std::string base_frame, std::string marker_topic = RVIZ_MARKER_TOPIC, rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("~"))
    : RvizVisualTools(base_frame, marker_topic, node), node_(node) {}

  bool publishFrictionCone(const Eigen::Vector3d& origin, const double& height, const Eigen::Vector3d& normal, const double& friction_coeff, rviz_visual_tools::Colors color)
  {
    double radius = friction_coeff * height;
    Eigen::Vector3d tail_end = origin + normal * height;

    // Set the frame ID and timestamp.
    // Use zero timestamp so RViz always uses the latest available TF and avoids future extrapolation.
    arrow_marker_.header.stamp = builtin_interfaces::msg::Time();
    arrow_marker_.header.frame_id = base_frame_;
    arrow_marker_.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker_.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker_.id++;

    arrow_marker_.points.clear();
    geometry_msgs::msg::Point p, start;
    p.x = tail_end(0);
    p.y = tail_end(1);
    p.z = tail_end(2);
    start.x = origin(0);
    start.y = origin(1);
    start.z = origin(2);
    arrow_marker_.pose.orientation.x = 0;
    arrow_marker_.pose.orientation.y = 0;
    arrow_marker_.pose.orientation.z = 0;
    arrow_marker_.pose.orientation.w = 1;
    arrow_marker_.points.resize(0);
    arrow_marker_.points.push_back(p);
    arrow_marker_.points.push_back(start);
    arrow_marker_.color = getColor(color);
    arrow_marker_.scale.x = 0.001;
    arrow_marker_.scale.y = 2 * radius;
    arrow_marker_.scale.z = height;

    // Helper for publishing rviz markers
    return publishMarker(arrow_marker_);
  }

  bool trigger()
  {
    // Force markers to use latest TF transform (stamp=0) to avoid timing races with TF publishers.
    for (auto& marker : markers_.markers)
    {
      marker.header.stamp = builtin_interfaces::msg::Time();
    }
    return RvizVisualTools::trigger();
  }
};

} // namespace rviz_visual_tools

namespace wolf_controller {

static std::mutex _mtx;

template <class MsgT>
class Visualizer
{
public:
  Visualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name, const std::string& base_frame = "world")
    : node_(node), cnt_(0), decimate_(10)
  {
    subscriber_ = node_->create_subscription<MsgT>(
          topic_name, 10, std::bind(&Visualizer::callback, this, std::placeholders::_1));
    std::string marker_topic = topic_name;
    if (!marker_topic.empty() && marker_topic.front() == '/')
      marker_topic.erase(0, 1);
    marker_topic += "_visual_marker";
    visual_tools_ = std::make_shared<rviz_visual_tools::VisualTools>(base_frame, marker_topic, node_);
  }

protected:
  virtual void callback(const typename MsgT::SharedPtr msg) = 0;

  void createCone(const geometry_msgs::msg::Vector3& normal, const geometry_msgs::msg::Vector3& position, double angle = M_PI)
  {
    vector_(0) = normal.x;
    vector_(1) = normal.y;
    vector_(2) = normal.z;
    norm_ = vector_.norm() + 0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), vector_ / norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = position.x;
    pose_.translation().y() = position.y;
    pose_.translation().z() = position.z;
    visual_tools_->publishCone(pose_, angle, rviz_visual_tools::Colors::LIME_GREEN, 0.05);
  }

  void createFrictionCone(const geometry_msgs::msg::Vector3& normal, const geometry_msgs::msg::Vector3& position, double friction_coeff)
  {
    vector_(0) = normal.x;
    vector_(1) = normal.y;
    vector_(2) = normal.z;
    norm_ = vector_.norm() + 0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), vector_ / norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = position.x;
    pose_.translation().y() = position.y;
    pose_.translation().z() = position.z;
    visual_tools_->publishFrictionCone(pose_.translation(), 0.05, vector_, friction_coeff, rviz_visual_tools::Colors::LIME_GREEN);
  }

  void createPlane(const geometry_msgs::msg::Vector3& normal, const geometry_msgs::msg::Vector3& position)
  {
    vector_(0) = normal.x;
    vector_(1) = normal.y;
    vector_(2) = normal.z;
    norm_ = vector_.norm() + 0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), vector_ / norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = position.x;
    pose_.translation().y() = position.y;
    pose_.translation().z() = position.z;
    visual_tools_->publishYZPlane(pose_);
  }

  void createArrow(const geometry_msgs::msg::Vector3& vector, const geometry_msgs::msg::Vector3& origin, rviz_visual_tools::Colors color, double scale = 500.0)
  {
    vector_(0) = vector.x;
    vector_(1) = vector.y;
    vector_(2) = vector.z;
    norm_ = vector_.norm() + 0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), vector_ / norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_ / scale);
  }

  void createArrow(const geometry_msgs::msg::Vector3& vector, const geometry_msgs::msg::Point& origin, rviz_visual_tools::Colors color, double scale = 500.0)
  {
    vector_(0) = vector.x;
    vector_(1) = vector.y;
    vector_(2) = vector.z;
    norm_ = vector_.norm() + 0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), vector_ / norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_ / scale);
  }

  void createPolygon(const geometry_msgs::msg::Polygon& poly, rviz_visual_tools::Colors color)
  {
    visual_tools_->publishPolygon(poly, color, rviz_visual_tools::LARGE);
  }

  void createSphere(const geometry_msgs::msg::Point& origin, rviz_visual_tools::Colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_, color, rviz_visual_tools::XXLARGE);
  }

  void createSphere(const geometry_msgs::msg::Vector3& origin, rviz_visual_tools::Colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_, color, rviz_visual_tools::XXLARGE);
  }

  rclcpp::Node::SharedPtr node_;
  long long cnt_;
  unsigned int decimate_;
  Eigen::Isometry3d pose_;
  Eigen::Vector3d vector_;
  double norm_;
  Eigen::Matrix3d R_;
  Eigen::Quaterniond q_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscriber_;
  rviz_visual_tools::VisualTools::Ptr visual_tools_;
};


// FrictionConesVisualizer Class
class FrictionConesVisualizer : public Visualizer<wolf_msgs::msg::FrictionCones>
{
public:
  using Ptr = std::shared_ptr<FrictionConesVisualizer>;

  FrictionConesVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::FrictionCones>(node, topic_name)
  {
  }

  virtual ~FrictionConesVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::FrictionCones::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      for (unsigned int i = 0; i < msg->foot_positions.size(); ++i)
      {
        createFrictionCone(msg->cone_axis[i], msg->foot_positions[i], static_cast<double>(msg->mus[i].data));
      }
      visual_tools_->trigger();
    }
  }
};

// TerrainEstimationVisualizer Class
class TerrainEstimationVisualizer : public Visualizer<wolf_msgs::msg::TerrainEstimation>
{
public:
  using Ptr = std::shared_ptr<TerrainEstimationVisualizer>;

  TerrainEstimationVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::TerrainEstimation>(node, topic_name)
  {
  }

  virtual ~TerrainEstimationVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::TerrainEstimation::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      createPlane(msg->terrain_normal, msg->central_point);
      createArrow(msg->terrain_normal, msg->central_point, rviz_visual_tools::CYAN, 10.0);
      visual_tools_->trigger();
    }
  }
};

// ContactForcesVisualizer Class
class ContactForcesVisualizer : public Visualizer<wolf_msgs::msg::ContactForces>
{
public:
  ContactForcesVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::ContactForces>(node, topic_name)
  {
  }

  virtual ~ContactForcesVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::ContactForces::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      for (unsigned int i = 0; i < msg->contact.size(); ++i)
      {
        createArrow(msg->des_contact_forces[i].force, msg->contact_positions[i], rviz_visual_tools::BLUE);
        createArrow(msg->contact_forces[i].force, msg->contact_positions[i], rviz_visual_tools::GREEN);
      }
      visual_tools_->trigger();
    }
  }
};

// CoMVisualizer Class
class CoMVisualizer : public Visualizer<wolf_msgs::msg::ComTask>
{
public:
  CoMVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::ComTask>(node, topic_name)
  {
  }

  virtual ~CoMVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::ComTask::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      wolf_msgs::msg::ComTask com_projection = *msg;
      com_projection.position_actual.z = 0.0;
      createSphere(com_projection.position_actual, rviz_visual_tools::RED);
      createSphere(msg->position_actual, rviz_visual_tools::GREEN);
      createArrow(msg->velocity_reference, msg->position_actual, rviz_visual_tools::BLUE, 1.0);
      visual_tools_->trigger();
    }
  }
};

// CapturePointVisualizer Class
class CapturePointVisualizer : public Visualizer<wolf_msgs::msg::CapturePoint>
{
public:
  CapturePointVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::CapturePoint>(node, topic_name)
  {
  }

  virtual ~CapturePointVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::CapturePoint::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      wolf_msgs::msg::CapturePoint data = *msg;
      createSphere(data.capture_point, rviz_visual_tools::RED);
      createSphere(data.com, rviz_visual_tools::GREEN);
      createPolygon(data.support_polygon, rviz_visual_tools::GREEN);
      visual_tools_->trigger();
    }
  }
};

// FootHoldsVisualizer Class
class FootHoldsVisualizer : public Visualizer<wolf_msgs::msg::FootHolds>
{
public:
  FootHoldsVisualizer(rclcpp::Node::SharedPtr node, const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::FootHolds>(node, topic_name)
  {
  }

  virtual ~FootHoldsVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::FootHolds::SharedPtr msg) override
  {
    if(cnt_++ % decimate_ == 0)
    {
      std::lock_guard<std::mutex> lock(_mtx);
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg->header.frame_id);
      for (unsigned int i = 0; i < msg->name.size(); ++i)
      {
        createSphere(msg->virtual_foothold[i], rviz_visual_tools::RED);
      }
      visual_tools_->trigger();
    }
  }
};

} // namespace wolf_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("wolf_controller", node_options);

  auto cfv = std::make_shared<wolf_controller::ContactForcesVisualizer>(node, "wolf_controller/contact_forces");
  auto comv = std::make_shared<wolf_controller::CoMVisualizer>(node, "wolf_controller/CoM");
  auto fhv = std::make_shared<wolf_controller::FootHoldsVisualizer>(node, "wolf_controller/foot_holds");
  auto tev = std::make_shared<wolf_controller::TerrainEstimationVisualizer>(node, "wolf_controller/terrain_estimation");
  auto fcv = std::make_shared<wolf_controller::FrictionConesVisualizer>(node, "wolf_controller/friction_cones");
  auto cpv = std::make_shared<wolf_controller::CapturePointVisualizer>(node, "wolf_controller/capture_point");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
