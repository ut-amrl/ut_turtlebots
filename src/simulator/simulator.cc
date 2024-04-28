//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    simulator.h
\brief   C++ Implementation: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/msg/pose2_d.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "simulator.h"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "config_reader/config_reader.h"
#include "math/geometry.h"
#include "math/line2d.h"
#include "math/math_util.h"
//#include "ros/ros_helpers.h"
#include "util/random.h"
#include "util/timer.h"
#include "vector_map.h"

#include <ros2_helpers.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

#include <geometry_msgs/msg/transform_stamped.hpp>

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::atan2;
using std::max;
using std::string;
using std::vector;
using vector_map::VectorMap;

DEFINE_bool(localize, false, "Publish localization");

CONFIG_STRING(cMapName, "map_name");
CONFIG_FLOAT(cCarLength, "car_length");
CONFIG_FLOAT(cCarWidth, "car_width");
CONFIG_FLOAT(cCarHeight, "car_height");
CONFIG_FLOAT(cRearAxleOffset, "rear_axle_offset");
CONFIG_FLOAT(cLaserLocX, "laser_loc.x");
CONFIG_FLOAT(cLaserLocY, "laser_loc.y");
CONFIG_FLOAT(cLaserLocZ, "laser_loc.z");
CONFIG_FLOAT(cStartX, "start_x");
CONFIG_FLOAT(cStartY, "start_y");
CONFIG_FLOAT(cStartAngle, "start_angle");
CONFIG_FLOAT(cPublishRate, "publish_rate");
CONFIG_FLOAT(cSubSampleRate, "sub_sample_rate");
CONFIG_FLOAT(cMaxAccel, "max_accel");
CONFIG_FLOAT(cMaxSpeed, "max_speed");
CONFIG_FLOAT(cLaserStdDev, "laser_noise_stddev");
CONFIG_FLOAT(cAngularDriftRate, "angular_drift_rate");
CONFIG_FLOAT(cAngularErrorRate, "angular_error_rate");
CONFIG_FLOAT(cMaxLaserRange, "max_laser_range");
CONFIG_FLOAT(cLaserAngleIncrement, "laser_angle_increment");
CONFIG_FLOAT(cLaserFOV, "laser_fov");
config_reader::ConfigReader reader({"config/simulator.lua"});

string MapNameToFile(const string &map) {
  string maps_dir_ = ament_index_cpp::get_package_share_directory("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

geometry_msgs::msg::TransformStamped populateTransformStampedMsg(
    const tf2::Transform &transform,
    const rclcpp::Time &time,
    const std::string &parent_frame,
    const std::string &child_frame) {
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = time;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;

  tf2::Vector3 transl = transform.getOrigin();
  t.transform.translation.x = transl.x();
  t.transform.translation.y = transl.y();
  t.transform.translation.z = transl.z();

  tf2::Quaternion quat = transform.getRotation();
  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();
  t.transform.rotation.w = quat.w();

  return t;
}

Simulator::Simulator() :
    random_(GetMonotonicTime() * 1e6),
    odom_loc_(random_.UniformRandom(-10, 10),
              random_.UniformRandom(-10, 10)),
    odom_angle_(random_.UniformRandom(-M_PI, M_PI)) {
  t_last_cmd_ = GetMonotonicTime();
  truePoseMsg.header.frame_id = "map";
}

Simulator::~Simulator() { }

void Simulator::ResetState() {
  odom_loc_ = Vector2f(random_.UniformRandom(-10, 10),
                       random_.UniformRandom(-10, 10));
  odom_angle_ = random_.UniformRandom(-M_PI, M_PI);
  true_robot_loc_ = Vector2f(cStartX, cStartY);
  true_robot_angle_ = cStartAngle;
  map_name_ = cMapName;
  map_.Load(MapNameToFile(cMapName));
}

void Simulator::Init(std::shared_ptr<rclcpp::Node>& n) {

  node_ = n;
  scan_msg_.header.frame_id = "base_laser";
  scan_msg_.angle_min = -0.5 * cLaserFOV;
  scan_msg_.angle_max = 0.5 * cLaserFOV;
  scan_msg_.range_min = 0.02;
  scan_msg_.range_max = cMaxLaserRange;
  scan_msg_.angle_increment = cLaserAngleIncrement;
  scan_msg_.intensities.clear();
  scan_msg_.time_increment = 0.0;
  scan_msg_.scan_time = 0.05;

  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_footprint";

  ResetState();
  InitSimulatorVizMarkers();
  DrawMap();

  drive_subscriber_ = n->create_subscription<geometry_msgs::msg::Twist>(
      "/ut/cmd_vel",
      1,
      std::bind(&Simulator::DriveCallback,
      this, std::placeholders::_1));
  init_subscriber_ = n->create_subscription<amrl_msgs::msg::Localization2DMsg>(
      "/set_pose", 1, std::bind(&Simulator::InitalLocationCallback, this, std::placeholders::_1));
  odometry_publisher_ = n->create_publisher<nav_msgs::msg::Odometry>("/odom",1);
  laser_publisher_ = n->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);
  map_publisher_ = n->create_publisher<visualization_msgs::msg::Marker>(
      "/simulator_visualization", 6);
  robot_marker_publisher_ = n->create_publisher<visualization_msgs::msg::Marker>(
      "/simulator_visualization", 6);
  true_pose_publisher_ = n->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/simulator_true_pose", 1);
  if (FLAGS_localize) {
    localization_publisher_ = n->create_publisher<amrl_msgs::msg::Localization2DMsg>(
        "/localization", 1);
    localization_msg_.header.frame_id = "map";
  }
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void Simulator::InitalLocationCallback(
    const amrl_msgs::msg::Localization2DMsg& msg) {
  true_robot_loc_ = Vector2f(msg.pose.x, msg.pose.y);
  true_robot_angle_ = msg.pose.theta;
  if (map_name_ != msg.map) {
    map_.Load(MapNameToFile(msg.map));
    map_name_ = msg.map;
    DrawMap();
  }
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0 @ %s\n",
         true_robot_loc_.x(),
         true_robot_loc_.y(),
         RadToDeg(true_robot_angle_),
         msg.map.c_str());
}


/**
 * Helper method that initializes visualization_msgs::msg::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::msg::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist,
 *                      linestrip, points
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::msg::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void Simulator::InitVizMarker(
    visualization_msgs::msg::Marker& vizMarker,
    string ns,
    int id,
    string type,
    geometry_msgs::msg::PoseStamped p,
    geometry_msgs::msg::Point32 scale,
    double duration,
    vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = node_->get_clock()->now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::msg::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::msg::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::msg::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::msg::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
  } else if (type == "linestrip") {
    vizMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  } else if (type == "points") {
    vizMarker.type = visualization_msgs::msg::Marker::POINTS;
  } else {
    vizMarker.type = visualization_msgs::msg::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  int32_t duration_sec = (int32_t) duration;
  uint32_t duration_nsec_component = (duration - duration_sec) * 1e9;

  vizMarker.lifetime = rclcpp::Duration(duration_sec, duration_nsec_component);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::msg::Marker::ADD;
}

void Simulator::InitSimulatorVizMarkers() {
  geometry_msgs::msg::PoseStamped p;
  geometry_msgs::msg::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.05;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  InitVizMarker(
      line_list_marker_, "map_lines", 0, "linelist", p, scale, 0.0, color);
  line_list_marker_.header.frame_id = "map";

  p.pose.position.z = 0.5 * cCarHeight;
  scale.x = cCarLength;
  scale.y = cCarWidth;
  scale.z = cCarHeight;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;
  InitVizMarker(
      robot_pos_marker_, "robot_position", 1, "cube", p, scale, 0.0, color);
  robot_pos_marker_.header.frame_id = "map";
}

void Simulator::DrawMap() {
  ros_helpers::ClearMarker(&line_list_marker_);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &line_list_marker_);
  }
}

void Simulator::PublishOdometry() {
  odom_msg_.header.stamp = node_->get_clock()->now();
  odom_msg_.pose.pose.position.x = odom_loc_.x();
  odom_msg_.pose.pose.position.y = odom_loc_.y();
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = sin(0.5 * odom_angle_);
  odom_msg_.pose.pose.orientation.w = cos(0.5 * odom_angle_);
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = robot_ang_vel_;
  odom_msg_.twist.twist.linear.x = robot_vel_;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0.0;

  odometry_publisher_->publish(odom_msg_);

  robot_pos_marker_.pose.position.x =
      true_robot_loc_.x() - cos(true_robot_angle_) * cRearAxleOffset;
  robot_pos_marker_.pose.position.y =
      true_robot_loc_.y() - sin(true_robot_angle_) * cRearAxleOffset;
  robot_pos_marker_.pose.position.z = 0.5 * cCarHeight;
  robot_pos_marker_.pose.orientation.x = 0;
  robot_pos_marker_.pose.orientation.y = 0;
  robot_pos_marker_.pose.orientation.z = sin(0.5 * true_robot_angle_);
  robot_pos_marker_.pose.orientation.w = cos(0.5 * true_robot_angle_);
}

void Simulator::PublishLaser() {
  scan_msg_.header.stamp = node_->get_clock()->now();
  const Vector2f laserRobotLoc(cLaserLocX, cLaserLocY);
  const Vector2f laserLoc = true_robot_loc_ + Rotation2Df(true_robot_angle_) * laserRobotLoc;

  const int num_rays = static_cast<int>(
      1.0 + (scan_msg_.angle_max - scan_msg_.angle_min) /
      scan_msg_.angle_increment);
  map_.GetPredictedScan(laserLoc,
                        scan_msg_.range_min,
                        scan_msg_.range_max,
                        scan_msg_.angle_min + true_robot_angle_,
                        scan_msg_.angle_max + true_robot_angle_,
                        num_rays,
                        &scan_msg_.ranges);
  for (float& r : scan_msg_.ranges) {
    if (r > scan_msg_.range_max - 0.1) {
      r = scan_msg_.range_max;
      continue;
    }
    r = max<float>(0.0, r + random_.Gaussian(0, cLaserStdDev));
  }
  laser_publisher_->publish(scan_msg_);
}

void Simulator::PublishTransform() {
  tf2::Transform transform;
  tf2::Quaternion q;

  geometry_msgs::msg::TransformStamped map_odom_tf;
  map_odom_tf.header.stamp =  node_->get_clock()->now();
  map_odom_tf.header.frame_id = "/map";
  map_odom_tf.child_frame_id = "/odom";
  transform.setOrigin(tf2::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));


  tf_broadcaster_->sendTransform(populateTransformStampedMsg(transform, node_->get_clock()->now(), "/map", "/odom"));

  transform.setOrigin(tf2::Vector3(true_robot_loc_.x(), true_robot_loc_.y(), 0.0));
  q.setRPY(0.0,0.0,true_robot_angle_);
  transform.setRotation(q);
  tf_broadcaster_->sendTransform(populateTransformStampedMsg(transform, node_->get_clock()->now(), "/odom",
"/base_footprint"));

  transform.setOrigin(tf2::Vector3(0.0 ,0.0, 0.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf_broadcaster_->sendTransform(populateTransformStampedMsg(transform, node_->get_clock()->now(),
"/base_footprint", "/base_link"));

  transform.setOrigin(tf2::Vector3(cLaserLocX, cLaserLocY, cLaserLocZ));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1));
  tf_broadcaster_->sendTransform(populateTransformStampedMsg(transform, node_->get_clock()->now(),
"/base_link", "/base_laser"));
}

void Simulator::PublishVisualizationMarkers() {
  map_publisher_->publish(line_list_marker_);
  robot_marker_publisher_->publish(robot_pos_marker_);
}

float AbsBound(float x, float bound) {
  if (x > 0.0 && x > bound) {
    return bound;
  } else if (x < 0.0 && x < -bound) {
    return -bound;
  }
  return x;
}

void Simulator::DriveCallback(const geometry_msgs::msg::Twist& msg) {
  if (!isfinite(msg.linear.x) || !isfinite(msg.angular.z)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg.linear.x,
           msg.angular.z);
    return;
  }
  if ((msg.linear.x < 0)) {
    printf("Ignoring negative linear velocity because turtlebot doesn't like backing up: %f\n",
           msg.linear.x);
  }
  last_cmd_ = msg;
  t_last_cmd_ = GetMonotonicTime();
}

void Simulator::Update() {
  static const double kMaxCommandAge = 0.1;
  if (!step_mode_ && GetMonotonicTime() > t_last_cmd_ + kMaxCommandAge) {
    last_cmd_.linear.x = 0;
  }
  const float dt = cSubSampleRate / cPublishRate;

  // Epsilon curvature corresponding to a very large radius of turning.
  static const float kEpsilonCurvature = 1.0 / 1E3;
  static const float kEpsilonLinearVel = 1.0 /1e3;

  // Commanded speed bounded to motion limit.
  const float desired_vel = AbsBound(last_cmd_.linear.x, cMaxSpeed);

  float dtheta;
  Vector2f dLoc;
  float dist = 0;
  if (desired_vel < kEpsilonLinearVel) {
    dLoc = Vector2f(0, 0);
    dtheta = dt * last_cmd_.angular.z;
    // Rotate in place
  } else {
    double last_curvature = last_cmd_.angular.z / last_cmd_.linear.x;

    // Commanded curvature bounded to turning limit.
    const float desired_curvature = last_curvature;
    // Indicates if the command is for linear motion.
    const bool linear_motion = (fabs(desired_curvature) < kEpsilonCurvature);

    const float dv_max = dt * cMaxAccel;
    const float bounded_dv = AbsBound(desired_vel - robot_vel_, dv_max);
    robot_vel_ = robot_vel_ + bounded_dv;
    dist = robot_vel_ * dt;

    // Robot-frame uncorrupted motion.
    if (linear_motion) {
      dLoc = Vector2f(dist, 0);
      dtheta = 0;
    } else {
      const float r = 1.0 / desired_curvature;
      dtheta = dist * desired_curvature;
      dLoc = r * Vector2f(sin(dtheta), 1.0 - cos(dtheta));
    }



  }

  odom_loc_ += Rotation2Df(odom_angle_) * dLoc;
  odom_angle_ = AngleMod(odom_angle_ + dtheta);

  true_robot_loc_ += Rotation2Df(true_robot_angle_) * dLoc;
  true_robot_angle_ =
      AngleMod(true_robot_angle_ + dtheta + dist * cAngularDriftRate +
               random_.Gaussian(0.0, fabs(dist) * cAngularErrorRate));

  truePoseMsg.header.stamp = node_->get_clock()->now();
  truePoseMsg.pose.position.x = true_robot_loc_.x();
  truePoseMsg.pose.position.y = true_robot_loc_.y();
  truePoseMsg.pose.position.z = 0;
  truePoseMsg.pose.orientation.w = cos(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.z = sin(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.x = 0;
  truePoseMsg.pose.orientation.y = 0;
  true_pose_publisher_->publish(truePoseMsg);
}

void Simulator::RunIteration() {
  // Simulate time-step.
  Update();

  if (last_publish_time_ < GetMonotonicTime() - 1.0 / cPublishRate) {
      //publish odometry and status
    PublishOdometry();
    //publish laser rangefinder messages
    PublishLaser();
    // publish visualization marker messages
    PublishVisualizationMarkers();
    //publish tf
    PublishTransform();
    last_publish_time_ = GetMonotonicTime();
  }

  if (FLAGS_localize) {
    localization_msg_.pose.x = true_robot_loc_.x();
    localization_msg_.pose.y = true_robot_loc_.y();
    localization_msg_.pose.theta = true_robot_angle_;
    localization_msg_.map = map_name_;
    localization_msg_.header.stamp = node_->get_clock()->now();
    localization_publisher_->publish(localization_msg_);
  }
}

void Simulator::Run() {
  // main loop
  const double simulator_fps = cPublishRate / cSubSampleRate;
  RateLoop rate(simulator_fps);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  while (rclcpp::ok()){
    executor.spin_once();
    RunIteration();
    rate.Sleep();
  }
}

void Simulator::Step(const geometry_msgs::msg::Twist& cmd,
                     nav_msgs::msg::Odometry* odom_msg,
                     sensor_msgs::msg::LaserScan* scan_msg,
                     amrl_msgs::msg::Localization2DMsg* localization_msg) {
  step_mode_ = true;
  DriveCallback(cmd);
  const int num_iterations = ceil(1.0 / cSubSampleRate);
  for (int i = 0; i < num_iterations; ++i) {
    RunIteration();
  }
  *odom_msg = odom_msg_;
  *scan_msg = scan_msg_;
  *localization_msg = localization_msg_;
}

void Simulator::SetStepMode(bool step_mode) {
  step_mode_ = step_mode;
}