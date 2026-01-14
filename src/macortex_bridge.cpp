/*=========================================================
//
// File: macortex_bridge.cpp
//
// Created by:   Ivan Ovinnikov, Aug-2013
// Modified by:  Lucas Waelti,   Apr-2024
// Ported to ROS 2 by: Kagan Erunsal, Dec-2025
//
// Description:
//   ROS 2 node for receiving pose data from the Motion Analysis Cortex system
//   and publishing it as standard ROS 2 messages.
//
// Notes on ROS 2 port:
//   - std_msgs/Header has no 'seq' field in ROS 2.
//   - The Cortex SDK invokes a C callback from its own thread; we protect shared
//     state with a mutex and publish from a timer.
//
// Topics (published):
//   - macortex_bridge/<body_name>/pose            (geometry_msgs/msg/PoseStamped)
//   - macortex_bridge/<body_name>/speed           (geometry_msgs/msg/Vector3Stamped) [optional]
//   - tf                                          (tf2_ros/TransformBroadcaster)
// =============================================================================*/

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <atomic>
#include <cctype>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include "cortex.h"

namespace
{
constexpr bool kEnableCortexLogging = false;
constexpr double kPi = 3.14159265269;

std::string sanitize_frame_id(std::string id)
{
  // TF frame_ids must not start with '/'. Many tools also expect frame IDs
  // to be URL/path-like but without spaces.
  while (!id.empty() && id.front() == '/') {
    id.erase(id.begin());
  }
  if (id.empty()) {
    return "frame";
  }

  for (char& c : id) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '-' || c == '/') {
      continue;
    }
    c = '_';
  }
  return id;
}

std::string make_child_frame_id(const std::string& node_namespace, const std::string& body_name)
{
  // Use namespace as a prefix to avoid collisions when multiple nodes run.
  std::string ns = node_namespace;
  while (!ns.empty() && ns.front() == '/') {
    ns.erase(ns.begin());
  }
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }

  std::string child = body_name;
  if (!ns.empty()) {
    child = ns + "/" + body_name;
  }
  return sanitize_frame_id(std::move(child));
}

std::string get_local_ip_for_target(const std::string& target)
{
  if (target.empty()) {
    return {};
  }

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  addrinfo* res = nullptr;
  // Port is irrelevant for the local-IP selection; any UDP port works.
  const int rc = getaddrinfo(target.c_str(), "7", &hints, &res);
  if (rc != 0 || res == nullptr) {
    return {};
  }

  int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(res);
    return {};
  }

  // UDP connect doesn't send packets; it just selects the outbound interface.
  (void)::connect(fd, res->ai_addr, res->ai_addrlen);

  sockaddr_in local{};
  socklen_t len = sizeof(local);
  std::string ip;
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&local), &len) == 0) {
    char buf[INET_ADDRSTRLEN] = {0};
    if (::inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf)) != nullptr) {
      ip = buf;
    }
  }

  ::close(fd);
  freeaddrinfo(res);
  return ip;
}

std::string get_default_outbound_ip()
{
  // Best-effort pick of the interface used for the default route.
  // This does not require actual internet connectivity.
  return get_local_ip_for_target("8.8.8.8");
}
}

class MacortexBridgeNode;
// Cortex SDK requires a C callback with no user-data. We route it to the active node instance.
// Use an atomic pointer because the callback is invoked from an SDK-owned thread.
static std::atomic<MacortexBridgeNode*> g_node{nullptr};

/**
 * @brief Convert roll, pitch, yaw angles into a quaternion
 */
Eigen::Quaterniond rpy2quat(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}
/**
 * @brief Error message handler for Cortex SDK
 * @param iLevel Error level
 * @param szMsg Error message from Cortex SDK
*/
void MyErrorMsgHandler(int iLevel, const char *szMsg)
{
  if(!kEnableCortexLogging) return;

  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }

  printf("    %s: %s\n", szLevel, szMsg);
}

class MacortexBridgeNode final : public rclcpp::Node
{
public:
  explicit MacortexBridgeNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("macortex_bridge", options)
  {
    declare_parameter<std::string>("cortex.my_ip", "");
    declare_parameter<std::string>("cortex.host_ip", "");

    declare_parameter<double>("rate", 30.0);
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<std::string>("prop_name", "");

    declare_parameter<bool>("enable_speed", false);
    declare_parameter<bool>("speed_to_body", false);
    declare_parameter<double>("cutoff", 10.0);

    rate_hz_ = get_parameter("rate").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    prop_name_ = get_parameter("prop_name").as_string();

    enable_speed_ = get_parameter("enable_speed").as_bool();
    speed_to_body_ = get_parameter("speed_to_body").as_bool();
    cutoff_hz_ = get_parameter("cutoff").as_double();

    cortex_my_ip_ = get_parameter("cortex.my_ip").as_string();
    cortex_host_ip_ = get_parameter("cortex.host_ip").as_string();

    // Publishes transforms for each tracked body on /tf.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "[macortex_bridge] Running with configuration:");
    RCLCPP_INFO(get_logger(), "- Poses expressed in '%s' frame at %.2f Hz", frame_id_.c_str(), rate_hz_);
    if (prop_name_.empty()) {
      RCLCPP_INFO(get_logger(), "- Publishing all MCS bodies");
    } else {
      RCLCPP_WARN(get_logger(), "- Publishing only MCS body: '%s'", prop_name_.c_str());
    }

    if (enable_speed_) {
      RCLCPP_INFO(get_logger(), "- Speed enabled, %s frame, cutoff freq. %.2f Hz",
        speed_to_body_ ? "body" : "inertial", cutoff_hz_);
    } else {
      RCLCPP_INFO(get_logger(), "- Speed disabled");
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_latest(); });
  }

  ~MacortexBridgeNode() override
  {
    shutdown_cortex();
  }

  bool init_cortex()
  {
    int retval = RC_Okay;
    unsigned char sdk_version[4];

    Cortex_SetVerbosityLevel(VL_Info);
    Cortex_GetSdkVersion(sdk_version);
    RCLCPP_INFO(get_logger(), "Using Cortex SDK Version %d.%d.%d", sdk_version[1], sdk_version[2], sdk_version[3]);

    Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
    // Cortex SDK calls this callback from its own networking thread.
    // We only buffer the latest data there; actual publishing is done from a ROS timer.
    Cortex_SetDataHandlerFunc(&MacortexBridgeNode::CortexDataHandler);

    // Cortex_Initialize expects:
    //   - my NIC address: IP of the local interface on the Cortex/MCS network
    //   - Cortex NIC address: hostname/IP of the MCS computer (optional; NULL auto-discovery)
    //
    // We provide ROS 2 params for both and keep ROS_IP as a fallback for my NIC.
    const char* env_ros_ip = std::getenv("ROS_IP");
    std::string my_ip_auto;
    const char* my_ip = nullptr;
    if (!cortex_my_ip_.empty()) {
      my_ip = cortex_my_ip_.c_str();
    } else if (env_ros_ip != nullptr && std::string(env_ros_ip).size() > 0) {
      my_ip = env_ros_ip;
    }

    const char* host_ip = cortex_host_ip_.empty() ? nullptr : cortex_host_ip_.c_str();

    // Auto-detect the local NIC IP if the user didn't provide one.
    // If host IP is provided, prefer the interface that would route to it.
    if (my_ip == nullptr) {
      if (host_ip != nullptr) {
        my_ip_auto = get_local_ip_for_target(host_ip);
      }
      if (my_ip_auto.empty()) {
        my_ip_auto = get_default_outbound_ip();
      }
      if (!my_ip_auto.empty()) {
        my_ip = my_ip_auto.c_str();
        RCLCPP_INFO(get_logger(), "Auto-detected local NIC IP for Cortex: %s", my_ip);
      }
    }

    if (my_ip == nullptr) {
      RCLCPP_WARN(get_logger(),
        "No local NIC IP configured (cortex.my_ip/ROS_IP). Cortex will auto-select an interface.");
    } else {
      RCLCPP_INFO(get_logger(), "Using local NIC IP for Cortex: %s", my_ip);
    }
    if (host_ip == nullptr) {
      RCLCPP_INFO(get_logger(), "Cortex host IP not set (cortex.host_ip). Using auto-discovery.");
    } else {
      RCLCPP_INFO(get_logger(), "Using Cortex host IP: %s", host_ip);
    }

    retval = Cortex_Initialize(my_ip, host_ip);
    if (retval != RC_Okay) {
      RCLCPP_ERROR(get_logger(), "Unable to initialize ethernet communication with Cortex");
      (void)Cortex_Exit();
      return false;
    }

    cortex_initialized_.store(true, std::memory_order_release);

    // Retry until we can query body defs and context frame rate, then switch to live mode.
    while (rclcpp::ok()) {
      sBodyDefs* body_defs = Cortex_GetBodyDefs();
      if (body_defs == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to get body defs, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      RCLCPP_INFO(get_logger(), "Got %d body defs", body_defs->nBodyDefs);
      // Helpful for configuring `prop_name`.
      for (int i = 0; i < body_defs->nBodyDefs; i++) {
        const char* name = body_defs->BodyDefs[i].szName;
        if (name != nullptr && std::string(name).size() > 0) {
          RCLCPP_INFO(get_logger(), "- BodyDef[%d]: %s", i, name);
        }
      }
      Cortex_FreeBodyDefs(body_defs);
      break;
    }

    void* response = nullptr;
    int n_bytes = 0;
    while (rclcpp::ok()) {
      retval = Cortex_Request("GetContextFrameRate", &response, &n_bytes);
      if (retval != RC_Okay) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GetContextFrameRate failed, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      float* context_frame_rate = static_cast<float*>(response);
      RCLCPP_INFO(get_logger(), "Cortex ContextFrameRate = %3.1f Hz", *context_frame_rate);
      break;
    }

    while (rclcpp::ok()) {
      retval = Cortex_Request("LiveMode", &response, &n_bytes);
      if (retval != RC_Okay) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "LiveMode failed, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      break;
    }

    RCLCPP_INFO(get_logger(), "[macortex_bridge] Cortex bridge is now running...");
    return true;
  }

  void shutdown_cortex()
  {
    // Make shutdown idempotent: it can be called from on_shutdown and destructor.
    const bool was_initialized = cortex_initialized_.exchange(false, std::memory_order_acq_rel);
    if (was_initialized) {
      (void)Cortex_Exit();
    }
  }

private:
  static void CortexDataHandler(sFrameOfData* frame)
  {
    if (auto* node = g_node.load(std::memory_order_acquire)) {
      node->on_cortex_frame(frame);
    }
  }

  void on_cortex_frame(sFrameOfData* frame)
  {
    if (frame == nullptr) {
      return;
    }

    const rclcpp::Time current_time = now();
    frames_received_.fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(mutex_);

    if (frame->nBodies <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cortex frame received but contains 0 bodies.");
    }

    int valid_pose_count = 0;
    for (int i = 0; i < frame->nBodies; i++) {
      const auto& body_data = frame->BodyData[i];
      const std::string body_name(body_data.szName);

      if (!prop_name_.empty() && body_name != prop_name_) {
        continue;
      }

      if (body_data.nSegments < 1 || body_data.Segments == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Body '%s' has no segment data (nSegments=%d). Cannot compute pose.",
          body_name.c_str(), body_data.nSegments);
        continue;
      }

      // Extract pose in MCS frame.
      // Cortex provides segment data in millimeters and degrees.
      const Eigen::Vector3d pos(
        body_data.Segments[0][0] / 1000.0,
        body_data.Segments[0][1] / 1000.0,
        body_data.Segments[0][2] / 1000.0);
      const Eigen::Quaterniond ori = rpy2quat(
        body_data.Segments[0][3] / 180.0 * kPi,
        body_data.Segments[0][4] / 180.0 * kPi,
        body_data.Segments[0][5] / 180.0 * kPi);

      // Cortex uses large sentinel values when invalid.
      if (pos.x() > 9000.0 || pos.y() > 9000.0 || pos.z() > 9000.0) {
        // If we are filtering for a single prop and its pose is invalid,
        // this will result in "node running but no messages".
        if (!prop_name_.empty() && body_name == prop_name_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Pose for '%s' is invalid (sentinel from Cortex). Check tracking / exact prop name.",
            body_name.c_str());
        }
        continue;
      }

      valid_pose_count++;

      // Create publishers lazily per body, but only once we have a valid pose.
      if (pose_pubs_.find(body_name) == pose_pubs_.end()) {
        const std::string pose_topic = "macortex_bridge/" + body_name + "/pose";
        pose_pubs_[body_name] = create_publisher<geometry_msgs::msg::PoseStamped>(
          pose_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

        RCLCPP_INFO(get_logger(), "[macortex_bridge] Advertising topics (%s prop):", body_name.c_str());
        RCLCPP_INFO(get_logger(), "\t- %s", pose_topic.c_str());

        if (enable_speed_) {
          const std::string speed_topic = "macortex_bridge/" + body_name + "/speed";
          speed_pubs_[body_name] = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            speed_topic, rclcpp::QoS(rclcpp::KeepLast(1)));
          RCLCPP_INFO(get_logger(), "\t- %s", speed_topic.c_str());
        }
      }

      geometry_msgs::msg::PoseStamped mcs_pose_msg;
      mcs_pose_msg.header.stamp = current_time;
      mcs_pose_msg.header.frame_id = frame_id_;
      mcs_pose_msg.pose.position.x = pos.x();
      mcs_pose_msg.pose.position.y = pos.y();
      mcs_pose_msg.pose.position.z = pos.z();
      mcs_pose_msg.pose.orientation.w = ori.w();
      mcs_pose_msg.pose.orientation.x = ori.x();
      mcs_pose_msg.pose.orientation.y = ori.y();
      mcs_pose_msg.pose.orientation.z = ori.z();

      if (enable_speed_ && last_pose_msgs_.find(body_name) != last_pose_msgs_.end()) {
        geometry_msgs::msg::Vector3Stamped mcs_speed_msg;
        mcs_speed_msg.header.stamp = current_time;
        mcs_speed_msg.header.frame_id = frame_id_;

        const rclcpp::Time prev_stamp(last_pose_msgs_[body_name].header.stamp);
        const double dt = (current_time - prev_stamp).seconds();
        if (dt > 1e-10) {
          const auto& prev_p = last_pose_msgs_[body_name].pose.position;
          mcs_speed_msg.vector.x = (pos.x() - prev_p.x) / dt;
          mcs_speed_msg.vector.y = (pos.y() - prev_p.y) / dt;
          mcs_speed_msg.vector.z = (pos.z() - prev_p.z) / dt;

          if (speed_to_body_) {
            mcs_speed_msg.header.frame_id = "base_link";
            Eigen::Vector3d v(mcs_speed_msg.vector.x, mcs_speed_msg.vector.y, mcs_speed_msg.vector.z);
            v = ori.conjugate() * v;
            mcs_speed_msg.vector.x = v.x();
            mcs_speed_msg.vector.y = v.y();
            mcs_speed_msg.vector.z = v.z();
          }

          if (cutoff_hz_ > 0.0 && last_speed_msgs_.find(body_name) != last_speed_msgs_.end()) {
            const double a = std::exp(-2.0 * kPi * cutoff_hz_ * dt);
            const auto& prev_v = last_speed_msgs_[body_name].vector;
            mcs_speed_msg.vector.x = a * prev_v.x + (1.0 - a) * mcs_speed_msg.vector.x;
            mcs_speed_msg.vector.y = a * prev_v.y + (1.0 - a) * mcs_speed_msg.vector.y;
            mcs_speed_msg.vector.z = a * prev_v.z + (1.0 - a) * mcs_speed_msg.vector.z;
          }

          last_speed_msgs_[body_name] = mcs_speed_msg;
        }
      }

      last_pose_msgs_[body_name] = mcs_pose_msg;
    }

    if (frame->nBodies > 0 && valid_pose_count == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cortex frame had %d bodies but 0 valid poses (all invalid/skipped).", frame->nBodies);
    }
  }

  void publish_latest()
  {
    // Lightweight health hint: if we never receive frames, nothing will publish.
    if (frames_received_.load(std::memory_order_relaxed) == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No streaming frames received from Cortex yet. If this persists, set cortex.my_ip and/or cortex.host_ip.");
    }

    const rclcpp::Time current_time = now();

    // Copy the latest state under lock, then publish outside the lock.
    // This keeps the Cortex callback thread unblocked and avoids mutating timestamps
    // used for speed finite-differencing.
    std::vector<std::pair<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr,
      geometry_msgs::msg::PoseStamped>> pose_jobs;
    std::vector<std::pair<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr,
      geometry_msgs::msg::Vector3Stamped>> speed_jobs;
    std::vector<geometry_msgs::msg::TransformStamped> tf_jobs;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      // Build TF transforms for all tracked bodies (independent of subscribers).
      // This mirrors the semantics of the aggregated poses topic (all bodies).
      tf_jobs.reserve(last_pose_msgs_.size());
      const std::string parent_frame = sanitize_frame_id(frame_id_);
      for (const auto& kv : last_pose_msgs_) {
        const auto& body_name = kv.first;
        const auto& pose_msg = kv.second;

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;
        tf.header.frame_id = parent_frame;
        tf.child_frame_id = make_child_frame_id(get_namespace(), body_name);
        tf.transform.translation.x = pose_msg.pose.position.x;
        tf.transform.translation.y = pose_msg.pose.position.y;
        tf.transform.translation.z = pose_msg.pose.position.z;
        tf.transform.rotation = pose_msg.pose.orientation;
        tf_jobs.emplace_back(std::move(tf));
      }

      pose_jobs.reserve(last_pose_msgs_.size());
      for (const auto& kv : last_pose_msgs_) {
        const auto& name = kv.first;
        const auto pub_it = pose_pubs_.find(name);
        if (pub_it == pose_pubs_.end()) {
          continue;
        }
        if (pub_it->second->get_subscription_count() == 0) {
          continue;
        }

        auto msg = kv.second;
        msg.header.stamp = current_time;
        pose_jobs.emplace_back(pub_it->second, std::move(msg));
      }

      if (enable_speed_) {
        speed_jobs.reserve(last_speed_msgs_.size());
        for (const auto& kv : last_speed_msgs_) {
          const auto& name = kv.first;
          const auto pub_it = speed_pubs_.find(name);
          if (pub_it == speed_pubs_.end()) {
            continue;
          }
          if (pub_it->second->get_subscription_count() == 0) {
            continue;
          }

          auto msg = kv.second;
          msg.header.stamp = current_time;
          speed_jobs.emplace_back(pub_it->second, std::move(msg));
        }
      }
    }

    for (auto& job : pose_jobs) {
      job.first->publish(job.second);
    }

    for (auto& job : speed_jobs) {
      job.first->publish(job.second);
    }

    if (!tf_jobs.empty()) {
      tf_broadcaster_->sendTransform(tf_jobs);
    }
  }

  // Parameters
  double rate_hz_ = 30.0;
  std::string frame_id_ = "world";
  std::string prop_name_;

  bool enable_speed_ = false;
  bool speed_to_body_ = false;
  double cutoff_hz_ = 10.0;

  std::string cortex_my_ip_;
  std::string cortex_host_ip_;

  // Publishers
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr> speed_pubs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State shared between Cortex callback thread and publish timer
  std::mutex mutex_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> last_pose_msgs_;
  std::unordered_map<std::string, geometry_msgs::msg::Vector3Stamped> last_speed_msgs_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::atomic<uint64_t> frames_received_{0};
  std::atomic<bool> cortex_initialized_{false};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MacortexBridgeNode>(rclcpp::NodeOptions{});
  g_node.store(node.get(), std::memory_order_release);

  if (!node->init_cortex()) {
    g_node.store(nullptr, std::memory_order_release);
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::on_shutdown([node]() {
    node->shutdown_cortex();
  });

  rclcpp::spin(node);
  g_node.store(nullptr, std::memory_order_release);
  rclcpp::shutdown();
  return 0;
}
