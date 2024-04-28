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
 * \file    websocket_main.cpp
 * \brief   Main entry point for websocket bridge.
 * \author  Joydeep Biswas, (C) 2019
 */
//========================================================================

#include <QtCore/QCoreApplication>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/math_util.h"
#include "rclcpp/rclcpp.hpp"
#include "util/timer.h"
#include "websocket/websocket.hpp"

using amrl_msgs::msg::Localization2DMsg;
using amrl_msgs::msg::VisualizationMsg;
using sensor_msgs::msg::LaserScan;
using std::vector;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

DEFINE_double(fps, 10.0, "Max visualization frames rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DECLARE_int32(v);

namespace {
bool run_ = true;
vector<VisualizationMsg> vis_msgs_;
geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg_;
geometry_msgs::msg::PoseStamped nav_goal_msg_;
amrl_msgs::msg::Localization2DMsg amrl_initial_pose_msg_;
amrl_msgs::msg::Localization2DMsg amrl_nav_goal_msg_;
Localization2DMsg localization_msg_;
LaserScan laser_scan_;

bool updates_pending_ = false;
RobotWebSocket *server_ = nullptr;
}  // namespace

class WebServerNode : public rclcpp::Node {
 public:
  WebServerNode() : Node("ut_web_server") {
    laser_sub_ = this->create_subscription<LaserScan>(
        "/scan", 5, std::bind(&WebServerNode::LaserCallback, this, _1));
    vis_sub_ = this->create_subscription<VisualizationMsg>(
        "/visualization",
        10,
        std::bind(&WebServerNode::VisualizationCallback, this, _1));
    localization_sub_ = this->create_subscription<Localization2DMsg>(
        "/localization",
        10,
        std::bind(&WebServerNode::LocalizationCallback, this, _1));
    init_loc_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10);
    amrl_init_loc_pub_ =
        this->create_publisher<amrl_msgs::msg::Localization2DMsg>("/set_pose",
                                                                  10);
    amrl_nav_goal_pub_ =
        this->create_publisher<amrl_msgs::msg::Localization2DMsg>(
            "/set_nav_target", 10);
  }

  void SetInitialPose(float x, float y, float theta, QString map) {
    if (FLAGS_v > 0) {
      printf("Set initial pose: %s %f,%f, %f\n",
             map.toStdString().c_str(),
             x,
             y,
             math_util::RadToDeg(theta));
    }
    initial_pose_msg_.header.stamp = get_clock()->now();
    initial_pose_msg_.pose.pose.position.x = x;
    initial_pose_msg_.pose.pose.position.y = y;
    initial_pose_msg_.pose.pose.orientation.w = cos(0.5 * theta);
    initial_pose_msg_.pose.pose.orientation.z = sin(0.5 * theta);
    init_loc_pub_->publish(initial_pose_msg_);
    amrl_initial_pose_msg_.header.stamp = get_clock()->now();
    amrl_initial_pose_msg_.map = map.toStdString();
    amrl_initial_pose_msg_.pose.x = x;
    amrl_initial_pose_msg_.pose.y = y;
    amrl_initial_pose_msg_.pose.theta = theta;

    LOG(INFO) << "Publishing initial loc msg with "
              << amrl_initial_pose_msg_.map << " "
              << amrl_initial_pose_msg_.pose.x << " "
              << amrl_initial_pose_msg_.pose.y << " "
              << amrl_initial_pose_msg_.pose.theta;
    amrl_init_loc_pub_->publish(amrl_initial_pose_msg_);
  }

  void SetNavGoal(float x, float y, float theta, QString map) {
    if (FLAGS_v > 0) {
      printf("Set nav goal: %s %f,%f, %f\n",
             map.toStdString().c_str(),
             x,
             y,
             math_util::RadToDeg(theta));
    }
    nav_goal_msg_.header.stamp = get_clock()->now();
    nav_goal_msg_.pose.position.x = x;
    nav_goal_msg_.pose.position.y = y;
    nav_goal_msg_.pose.orientation.w = cos(0.5 * theta);
    nav_goal_msg_.pose.orientation.z = sin(0.5 * theta);
    nav_goal_pub_->publish(nav_goal_msg_);
    amrl_nav_goal_msg_.header.stamp = get_clock()->now();
    amrl_nav_goal_msg_.map = map.toStdString();
    amrl_nav_goal_msg_.pose.x = x;
    amrl_nav_goal_msg_.pose.y = y;
    amrl_nav_goal_msg_.pose.theta = theta;
    LOG(INFO) << "Publishing nav goal with " << amrl_nav_goal_msg_.map << " "
              << amrl_nav_goal_msg_.pose.x << " " << amrl_nav_goal_msg_.pose.y
              << " " << amrl_nav_goal_msg_.pose.theta;
    amrl_nav_goal_pub_->publish(amrl_nav_goal_msg_);
  }

  void LocalizationCallback(const Localization2DMsg &msg) {
    localization_msg_ = msg;
  }

  void LaserCallback(const LaserScan &msg) {
    laser_scan_ = msg;
    updates_pending_ = true;
  }

  void VisualizationCallback(const VisualizationMsg &msg) {
    static bool warning_showed_ = false;
    if (msg.header.frame_id != "base_link" && msg.header.frame_id != "map") {
      if (!warning_showed_) {
        fprintf(stderr,
                "WARNING: Ignoring visualization for unknown frame '%s'."
                " This message prints only once.\n",
                msg.header.frame_id.c_str());
        warning_showed_ = true;
      }
      return;
    }
    auto prev_msg = std::find_if(
        vis_msgs_.begin(), vis_msgs_.end(), [&msg](const VisualizationMsg &m) {
          return m.ns == msg.ns;
        });
    if (prev_msg == vis_msgs_.end()) {
      vis_msgs_.push_back(msg);
    } else {
      *prev_msg = msg;
    }
    updates_pending_ = true;
  }

  // void DropOldMessages()
  // {
  //   const auto now = get_clock()->now();
  //   if ((now - laser_scan_.header.stamp).to_chrono<std::chrono::seconds>() >
  //   std::chrono::seconds(FLAGS_max_age))
  //   {
  //     laser_scan_.header.stamp = rclcpp::Time(0);
  //   }
  //   std::remove_if(
  //       vis_msgs_.begin(),
  //       vis_msgs_.end(),
  //       [&now](const VisualizationMsg &m)
  //       {
  //         return ((now - m.header.stamp).to_chrono<std::chrono::seconds>() >
  //         std::chrono::seconds(FLAGS_max_age));
  //       });
  // }

  void DropOldMessages() {
    using double_seconds = std::chrono::duration<double>;

    // Current time
    const auto now = get_clock()->now();

    // Duration since the last laser scan
    auto laser_duration = (now - laser_scan_.header.stamp)
                              .to_chrono<std::chrono::duration<double>>();

    // Drop laser scan if it exceeds the max age
    if (laser_duration > double_seconds(FLAGS_max_age)) {
      laser_scan_.header.stamp = rclcpp::Time(0);
    }

    // Remove old visualization messages exceeding max age
    std::remove_if(
        vis_msgs_.begin(), vis_msgs_.end(), [&now](const VisualizationMsg &m) {
          auto vis_duration =
              (now - m.header.stamp).to_chrono<std::chrono::duration<double>>();
          return vis_duration > double_seconds(FLAGS_max_age);
        });
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_loc_pub_;
  rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr
      amrl_init_loc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr
      amrl_nav_goal_pub_;

  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<VisualizationMsg>::SharedPtr vis_sub_;
  rclcpp::Subscription<Localization2DMsg>::SharedPtr localization_sub_;
};

template <typename T>
void MergeVector(const std::vector<T> &v1, std::vector<T> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge message m1 into m2.
void MergeMessage(const VisualizationMsg &m1, VisualizationMsg *m2_ptr) {
  VisualizationMsg &m2 = *m2_ptr;
  MergeVector(m1.points, &m2.points);
  MergeVector(m1.lines, &m2.lines);
  MergeVector(m1.arcs, &m2.arcs);
  MergeVector(m1.text_annotations, &m2.text_annotations);
}

void SendUpdate() {
  if (server_ == nullptr || !updates_pending_) {
    return;
  }
  // DropOldMessages();
  updates_pending_ = false;
  if (laser_scan_.header.stamp.sec == 0 &&
      laser_scan_.header.stamp.nanosec == 0 && vis_msgs_.empty()) {
    return;
  }
  VisualizationMsg local_msgs;
  VisualizationMsg global_msgs;
  for (const VisualizationMsg &m : vis_msgs_) {
    // std::cout << m << std::endl;
    if (m.header.frame_id == "map") {
      MergeMessage(m, &global_msgs);
    } else {
      MergeMessage(m, &local_msgs);
    }
  }
  server_->Send(local_msgs, global_msgs, laser_scan_, localization_msg_);
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  FLAGS_colorlogtostderr = true;
  rclcpp::init(argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  QCoreApplication a(argc, argv);
  server_ = new RobotWebSocket(10272);
  QObject::connect(
      server_, &RobotWebSocket::closed, &a, &QCoreApplication::quit);

  std::shared_ptr<WebServerNode> node = std::make_shared<WebServerNode>();

  auto spin_executor = [&node, &server_, &run_]() {
    std::function<void(float x, float y, float theta, QString map)>
        set_initial_pose_func =
            std::bind(&WebServerNode::SetInitialPose, node, _1, _2, _3, _4);

    std::function<void(float x, float y, float theta, QString map)>
        set_nav_goal_func =
            std::bind(&WebServerNode::SetNavGoal, node, _1, _2, _3, _4);
    QObject::connect(
        server_, &RobotWebSocket::SetInitialPoseSignal, set_initial_pose_func);
    QObject::connect(
        server_, &RobotWebSocket::SetNavGoalSignal, set_nav_goal_func);

    RateLoop loop(FLAGS_fps);
    while (rclcpp::ok() && run_) {
      rclcpp::spin_some(node);
      // Update rate is throttled by the rate loop timer.
      SendUpdate();
      loop.Sleep();
    }
  };

  std::thread spinner_thread(spin_executor);

  const int retval = a.exec();
  run_ = false;

  spinner_thread.join();
  return retval;
}
