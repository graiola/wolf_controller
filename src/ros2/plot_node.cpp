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

namespace rviz_visual_tools {

class VisualTools : public RvizVisualTools
{
public:
  using Ptr = std::shared_ptr<VisualTools>;

  VisualTools(std::string base_frame, std::string marker_topic = RVIZ_MARKER_TOPIC, rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("~"))
    : RvizVisualTools(base_frame, marker_topic, node) {}

  bool publishFrictionCone(const Eigen::Vector3d& origin, const double& height, const Eigen::Vector3d& normal, const double& friction_coeff, colors color)
  {
    double radius = friction_coeff * height;
    Eigen::Vector3d tail_end = origin + normal * height;

    // Set the frame ID and timestamp.
    arrow_marker_.header.stamp = this->now();
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
};

} // namespace rviz_visual_tools

namespace wolf_controller {

static std::mutex _mtx;

template <class MsgT>
class Visualizer : public rclcpp::Node
{
public:
  Visualizer(const std::string& topic_name, const std::string& base_frame = "world")
    : Node("visualizer"), cnt_(0), decimate_(10)
  {
    subscription_ = this->create_subscription<MsgT>(topic_name, 10, std::bind(&Visualizer::callback, this, std::placeholders::_1));
    visual_tools_ = std::make_shared<rviz_visual_tools::VisualTools>(base_frame, topic_name + "_visual_marker", this->shared_from_this());
  }

protected:
  virtual void callback(const MsgT& msg) = 0;

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
    visual_tools_->publishCone(pose_, angle, rviz_visual_tools::colors::LIME_GREEN, 0.05);
    visual_tools_->trigger();
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
    visual_tools_->publishFrictionCone(pose_.translation(), 0.05, vector_, friction_coeff, rviz_visual_tools::colors::LIME_GREEN);
    visual_tools_->trigger();
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
    visual_tools_->trigger();
  }

  void createArrow(const geometry_msgs::msg::Vector3& vector, const geometry_msgs::msg::Vector3& origin, rviz_visual_tools::colors color, double scale = 500.0)
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
    visual_tools_->trigger();
  }

  void createArrow(const geometry_msgs::msg::Vector3& vector, const geometry_msgs::msg::Point& origin, rviz_visual_tools::colors color, double scale = 500.0)
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
    visual_tools_->trigger();
  }

  void createPolygon(const geometry_msgs::msg::Polygon& poly, rviz_visual_tools::colors color)
  {
    visual_tools_->publishPolygon(poly, color, rviz_visual_tools::LARGE);
    visual_tools_->trigger();
  }

  void createSphere(const geometry_msgs::msg::Point& origin, rviz_visual_tools::colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_, color, rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  void createSphere(const geometry_msgs::msg::Vector3& origin, rviz_visual_tools::colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_, color, rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  long long cnt_;
  unsigned int decimate_;
  Eigen::Isometry3d pose_;
  Eigen::Vector3d vector_;
  double norm_;
  Eigen::Matrix3d R_;
  Eigen::Quaterniond q_;
  rclcpp::Subscription<MsgT>::SharedPtr subscription_;
  rviz_visual_tools::VisualTools::Ptr visual_tools_;
};

class FrictionConesVisualizer : public Visualizer<wolf_msgs::msg::FrictionCones>
{
public:
  using Ptr = std::shared_ptr<FrictionConesVisualizer>;

  FrictionConesVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::FrictionCones>(topic_name)
  {
  }

  virtual ~FrictionConesVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::FrictionCones &msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      for (unsigned int i = 0; i < msg.foot_positions.size(); ++i)
      {
        createFrictionCone(msg.cone_axis[i], msg.foot_positions[i], static_cast<double>(msg.mus[i].data));
      }
      _mtx.unlock();
    }
  }
};

class TerrainEstimationVisualizer : public Visualizer<wolf_msgs::msg::TerrainEstimation>
{
public:
  using Ptr = std::shared_ptr<TerrainEstimationVisualizer>;

  TerrainEstimationVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::TerrainEstimation>(topic_name)
  {
  }

  virtual ~TerrainEstimationVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::TerrainEstimation &msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      createPlane(msg.terrain_normal, msg.central_point);
      createArrow(msg.terrain_normal, msg.central_point, rviz_visual_tools::CYAN, 10.0);
      _mtx.unlock();
    }
  }
};

class ContactForcesVisualizer : public Visualizer<wolf_msgs::msg::ContactForces>
{
public:
  ContactForcesVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::ContactForces>(topic_name)
  {
  }

  virtual ~ContactForcesVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::ContactForces &msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      for (unsigned int i = 0; i < msg.contact.size(); ++i)
      {
        createArrow(msg.des_contact_forces[i].force, msg.contact_positions[i], rviz_visual_tools::BLUE);
        createArrow(msg.contact_forces[i].force, msg.contact_positions[i], rviz_visual_tools::GREEN);
      }
      _mtx.unlock();
    }
  }
};

class CoMVisualizer : public Visualizer<wolf_msgs::msg::ComTask>
{
public:
  CoMVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::ComTask>(topic_name)
  {
  }

  virtual ~CoMVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::ComTask& msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      wolf_msgs::msg::ComTask com_projection = msg;
      com_projection.position_actual.z = 0.0;
      createSphere(com_projection.position_actual, rviz_visual_tools::RED);
      createSphere(msg.position_actual, rviz_visual_tools::GREEN);
      createArrow(msg.velocity_reference, msg.position_actual, rviz_visual_tools::BLUE, 1.0);
      _mtx.unlock();
    }
  }
};

class CapturePointVisualizer : public Visualizer<wolf_msgs::msg::CapturePoint>
{
public:
  CapturePointVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::CapturePoint>(topic_name)
  {
  }

  virtual ~CapturePointVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::CapturePoint& msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      wolf_msgs::msg::CapturePoint data = msg;
      createSphere(data.capture_point, rviz_visual_tools::RED);
      createSphere(data.com, rviz_visual_tools::GREEN);
      createPolygon(data.support_polygon, rviz_visual_tools::GREEN);
      _mtx.unlock();
    }
  }
};

class FootHoldsVisualizer : public Visualizer<wolf_msgs::msg::FootHolds>
{
public:
  FootHoldsVisualizer(const std::string& topic_name)
    : Visualizer<wolf_msgs::msg::FootHolds>(topic_name)
  {
  }

  virtual ~FootHoldsVisualizer() {}

protected:
  void callback(const wolf_msgs::msg::FootHolds& msg) override
  {
    if(cnt_++ % decimate_ == 0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      for (unsigned int i = 0; i < msg.name.size(); ++i)
      {
        createSphere(msg.virtual_foothold[i], rviz_visual_tools::RED);
      }
      _mtx.unlock();
    }
  }
};

} // namespace wolf_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<wolf_controller::Visualizer<void>>("wolf_controller");

  auto cfv = std::make_shared<wolf_controller::ContactForcesVisualizer>("contact_forces");
  auto comv = std::make_shared<wolf_controller::CoMVisualizer>("CoM");
  auto fhv = std::make_shared<wolf_controller::FootHoldsVisualizer>("foot_holds");
  auto tev = std::make_shared<wolf_controller::TerrainEstimationVisualizer>("terrain_estimation");
  auto fcv = std::make_shared<wolf_controller::FrictionConesVisualizer>("friction_cones");
  auto cpv = std::make_shared<wolf_controller::CapturePointVisualizer>("capture_point");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
