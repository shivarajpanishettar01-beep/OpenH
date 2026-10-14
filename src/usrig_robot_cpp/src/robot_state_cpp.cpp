// robot_state_cpp: UR7e state at 500 Hz over read-only RTDE, without losing samples.
//
// RTDE is started WITHOUT ur_client_library's background thread (start(false)): in that mode
// getDataPackage() would return only the latest packet and silently drop the ones in between when
// this thread is late. Instead getDataPackageBlocking() reads every packet in order from the TCP
// stream; a late wake-up delays samples (they wait in the socket buffer) but never skips them. Each sample gets ONE system-clock stamp, shared by /robot/state,
// /joint_states, /robot/tcp_pose and /robot/wrench. No forward kinematics: TCP pose is the
// controller's actual_TCP_pose (rotation vector -> quaternion).
//
// mode / safety_flags / action_source / action_is_expert / force_setpoint_n: same rule as
// usrig_freedrive/labels.py (keep in step):
//   * robot_control's /robot/control_status if younger than 50 ms (never carried forward);
//   * else freedrive: the configurable input wired to the foot pedal (UR installation: "Freedrive")
//     is held and robot_mode is RUNNING -> FREEDRIVE / KINESTHETIC / expert;
//   * else IDLE / NONE / not expert. Without robot_control the force setpoint is NaN.
// While robot_control is not commanding, /robot/cmd_executed carries NaN twists at 125 Hz: an explicit
// "no command" marker (the converter needs the topic; nothing is invented).
//
// Parameters: robot_ip, frequency_hz (500), rt_priority (0 = off), base_frame ("base"),
//             T_base_phantom (16 doubles, row-major; identity until phantom registration),
//             freedrive_input (configurable digital input 0-7 of the pedal; -1 = none).
#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/rtde/rtde_client.h>

#include "usrig_interfaces/msg/control_status.hpp"
#include "usrig_interfaces/msg/robot_state.hpp"

using RobotState = usrig_interfaces::msg::RobotState;
using ControlStatus = usrig_interfaces::msg::ControlStatus;
using Mat4 = std::array<double, 16>;

namespace
{
const std::vector<std::string> kJointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const std::vector<std::string> kOutputs = {"timestamp", "actual_q", "actual_qd", "actual_TCP_pose",
                                           "actual_TCP_speed", "actual_TCP_force",
                                           "actual_digital_input_bits", "robot_mode"};
constexpr double kControlStatusMaxAgeS = 0.05;  // labels.py CONTROL_STATUS_MAX_AGE_S
constexpr int kConfigurableInputBit0 = 8;       // actual_digital_input_bits: 8-15 = configurable inputs
constexpr int32_t kRobotModeRunning = 7;

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time to_stamp(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

// rotation vector -> quaternion (x, y, z, w)
std::array<double, 4> rotvec_to_quat(double rx, double ry, double rz)
{
  const double a = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (a < 1e-12) {
    return {0.5 * rx, 0.5 * ry, 0.5 * rz, 1.0};
  }
  const double s = std::sin(0.5 * a) / a;
  return {rx * s, ry * s, rz * s, std::cos(0.5 * a)};
}

// quaternion -> 3x3 rotation (row-major)
std::array<double, 9> quat_to_rot(const std::array<double, 4>& q)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
          2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

std::array<double, 4> rot_to_quat(const std::array<double, 9>& R)
{
  std::array<double, 4> q{};
  const double tr = R[0] + R[4] + R[8];
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    q = {(R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s, 0.25 * s};
  } else if (R[0] > R[4] && R[0] > R[8]) {
    const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2;
    q = {0.25 * s, (R[1] + R[3]) / s, (R[2] + R[6]) / s, (R[7] - R[5]) / s};
  } else if (R[4] > R[8]) {
    const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2;
    q = {(R[1] + R[3]) / s, 0.25 * s, (R[5] + R[7]) / s, (R[2] - R[6]) / s};
  } else {
    const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2;
    q = {(R[2] + R[6]) / s, (R[5] + R[7]) / s, 0.25 * s, (R[3] - R[1]) / s};
  }
  return q;
}

void fill_pose(geometry_msgs::msg::Pose& p, const double xyz[3], const std::array<double, 4>& q)
{
  p.position.x = xyz[0];
  p.position.y = xyz[1];
  p.position.z = xyz[2];
  p.orientation.x = q[0];
  p.orientation.y = q[1];
  p.orientation.z = q[2];
  p.orientation.w = q[3];
}
}  // namespace

class RobotStateCpp : public rclcpp::Node
{
public:
  RobotStateCpp() : Node("robot_state")
  {
    ip_ = declare_parameter<std::string>("robot_ip", "");
    freq_ = declare_parameter<double>("frequency_hz", 500.0);
    rt_priority_ = static_cast<int>(declare_parameter<int64_t>("rt_priority", 0));
    frame_ = declare_parameter<std::string>("base_frame", "base");
    freedrive_input_ = static_cast<int>(declare_parameter<int64_t>("freedrive_input", -1));
    if (freedrive_input_ > 7) {
      throw std::runtime_error("freedrive_input must be a configurable digital input 0-7 (or -1)");
    }
    auto T = declare_parameter<std::vector<double>>("T_base_phantom", std::vector<double>{});
    if (ip_.empty()) {
      throw std::runtime_error("robot_ip parameter is empty (rig.yaml robot.ip / ursim_ip)");
    }
    set_phantom(T);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(500)).reliable();  // 1 s at 500 Hz
    pub_state_ = create_publisher<RobotState>("/robot/state", qos);
    pub_js_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot/tcp_pose", qos);
    pub_wrench_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/robot/wrench", qos);
    pub_nocmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/robot/cmd_executed", 10);
    sub_status_ = create_subscription<ControlStatus>(
        "/robot/control_status", rclcpp::QoS(10).reliable(), [this](ControlStatus::SharedPtr m) {
          std::lock_guard<std::mutex> lk(status_mtx_);
          status_ = *m;
          have_status_ = true;
          status_t_ns_ = steady_ns();
        });
    status_.mode = RobotState::MODE_IDLE;
    status_.action_source = RobotState::SOURCE_NONE;

    client_ = std::make_unique<urcl::rtde_interface::RTDEClient>(ip_, notifier_, kOutputs,
                                                                 std::vector<std::string>{}, freq_);
    if (!client_->init()) {
      throw std::runtime_error("RTDE init failed for " + ip_);
    }
    if (!client_->start(false)) {  // no background thread: lossless blocking reads
      throw std::runtime_error("RTDE start failed for " + ip_);
    }
    RCLCPP_INFO(get_logger(), "RTDE (ur_client_library) connected to %s at %.0f Hz (read-only)", ip_.c_str(),
                freq_);
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { log_stats(); });
    reader_ = std::thread([this] { run(); });
  }

  ~RobotStateCpp() override
  {
    stop_ = true;
    if (reader_.joinable()) {
      reader_.join();
    }
    if (client_) {
      client_->pause();
    }
  }

private:
  void set_phantom(const std::vector<double>& T)
  {
    if (T.size() == 16) {
      // store the inverse of T_base_phantom (rigid): R^T, -R^T t
      const double R[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]};
      const double t[3] = {T[3], T[7], T[11]};
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          inv_phantom_[i * 4 + j] = R[j * 3 + i];
        }
        inv_phantom_[i * 4 + 3] = -(R[0 * 3 + i] * t[0] + R[1 * 3 + i] * t[1] + R[2 * 3 + i] * t[2]);
      }
      inv_phantom_[12] = inv_phantom_[13] = inv_phantom_[14] = 0.0;
      inv_phantom_[15] = 1.0;
      have_phantom_ = true;
    } else if (!T.empty()) {
      throw std::runtime_error("T_base_phantom must have 16 values");
    }
  }

  void set_rt()
  {
    if (rt_priority_ <= 0) {
      return;
    }
    sched_param sp{};
    sp.sched_priority = rt_priority_;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
      RCLCPP_WARN(get_logger(),
                  "could not set SCHED_FIFO %d (%s); running at normal priority. Allow it with an rtprio "
                  "limit (docs/capture_pc_setup.md)",
                  rt_priority_, std::strerror(rc));
    } else {
      RCLCPP_INFO(get_logger(), "reader thread: SCHED_FIFO %d", rt_priority_);
    }
  }

  void run()
  {
    set_rt();
    const double period = 1.0 / freq_;
    auto pkg = std::make_unique<urcl::rtde_interface::DataPackage>(kOutputs);  // pre-allocated, reused
    while (!stop_ && rclcpp::ok()) {
      if (!client_->getDataPackageBlocking(pkg)) {
        ++read_failures_;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // connection problem: do not spin
        continue;
      }
      const int64_t stamp_ns = now_ns();  // one stamp per sample, taken at reception
      double ctrl_t = 0;
      urcl::vector6d_t q{}, qd{}, pose{}, speed{}, force{};
      uint64_t di_bits = 0;
      int32_t robot_mode = -1;
      pkg->getData("timestamp", ctrl_t);
      pkg->getData("actual_q", q);
      pkg->getData("actual_qd", qd);
      pkg->getData("actual_TCP_pose", pose);
      pkg->getData("actual_TCP_speed", speed);
      pkg->getData("actual_TCP_force", force);
      pkg->getData("actual_digital_input_bits", di_bits);
      pkg->getData("robot_mode", robot_mode);
      (void)speed;
      account(ctrl_t, stamp_ns, period);
      publish(stamp_ns, ctrl_t, q, qd, pose, force, di_bits, robot_mode);
    }
  }

  void account(double ctrl_t, int64_t stamp_ns, double period)
  {
    std::lock_guard<std::mutex> lk(stats_mtx_);
    ++samples_;
    if (have_last_) {
      const double dt = ctrl_t - last_ctrl_;
      if (dt <= 0) {
        ++repeated_;
      } else if (dt > 1.5 * period) {
        ++gaps_;
        lost_ += static_cast<long>(std::lround(dt / period)) - 1;
      }
    }
    last_ctrl_ = ctrl_t;
    have_last_ = true;
    stamp_minus_ctrl_.push_back(stamp_ns * 1e-9 - ctrl_t);
  }

  bool freedrive_engaged(uint64_t di_bits, int32_t robot_mode) const
  {
    if (freedrive_input_ < 0 || robot_mode != kRobotModeRunning) {
      return false;
    }
    return ((di_bits >> (kConfigurableInputBit0 + freedrive_input_)) & 1ULL) != 0;
  }

  // labels.py label_sample(); returns true if robot_control's status is in use
  bool label_sample(RobotState& m, uint64_t di_bits, int32_t robot_mode)
  {
    std::lock_guard<std::mutex> lk(status_mtx_);
    if (have_status_ && (steady_ns() - status_t_ns_) * 1e-9 <= kControlStatusMaxAgeS) {
      m.force_setpoint_n = status_.force_setpoint_n;
      m.mode = status_.mode;
      m.safety_flags = status_.safety_flags;
      m.action_source = status_.action_source;
      m.action_is_expert = status_.action_is_expert;
      return true;
    }
    m.force_setpoint_n = std::numeric_limits<double>::quiet_NaN();
    m.safety_flags = 0;
    if (freedrive_engaged(di_bits, robot_mode)) {
      m.mode = RobotState::MODE_FREEDRIVE;
      m.action_source = RobotState::SOURCE_KINESTHETIC;
      m.action_is_expert = true;
      ++freedrive_samples_;
    } else {
      m.mode = RobotState::MODE_IDLE;
      m.action_source = RobotState::SOURCE_NONE;
      m.action_is_expert = false;
    }
    return false;
  }

  void publish(int64_t stamp_ns, double ctrl_t, const urcl::vector6d_t& q, const urcl::vector6d_t& qd,
               const urcl::vector6d_t& pose, const urcl::vector6d_t& force, uint64_t di_bits,
               int32_t robot_mode)
  {
    const auto stamp = to_stamp(stamp_ns);
    const auto quat = rotvec_to_quat(pose[3], pose[4], pose[5]);
    const double xyz[3] = {pose[0], pose[1], pose[2]};

    auto m = std::make_unique<RobotState>();
    m->header.stamp = stamp;
    m->header.frame_id = frame_;
    m->controller_time_s = ctrl_t;
    for (int i = 0; i < 6; ++i) {
      m->q[i] = q[i];
      m->qd[i] = qd[i];
      m->wrench[i] = force[i];
    }
    fill_pose(m->tcp_base, xyz, quat);
    if (have_phantom_) {
      const auto R = quat_to_rot(quat);
      std::array<double, 9> Rp{};
      double p[3];
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          Rp[i * 3 + j] = inv_phantom_[i * 4 + 0] * R[0 * 3 + j] + inv_phantom_[i * 4 + 1] * R[1 * 3 + j] +
                          inv_phantom_[i * 4 + 2] * R[2 * 3 + j];
        }
        p[i] = inv_phantom_[i * 4 + 0] * xyz[0] + inv_phantom_[i * 4 + 1] * xyz[1] +
               inv_phantom_[i * 4 + 2] * xyz[2] + inv_phantom_[i * 4 + 3];
      }
      fill_pose(m->tcp_phantom, p, rot_to_quat(Rp));
    } else {
      fill_pose(m->tcp_phantom, xyz, quat);  // identity until phantom registration exists
    }
    const bool controlled = label_sample(*m, di_bits, robot_mode);
    std::unique_ptr<geometry_msgs::msg::TwistStamped> nocmd;
    if (!controlled && (++nocmd_counter_ % 4) == 0) {  // 125 Hz "no command" marker (NaN)
      nocmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
      nocmd->header.stamp = stamp;
      nocmd->header.frame_id = "none";
      const double nan = std::numeric_limits<double>::quiet_NaN();
      nocmd->twist.linear.x = nocmd->twist.linear.y = nocmd->twist.linear.z = nan;
      nocmd->twist.angular.x = nocmd->twist.angular.y = nocmd->twist.angular.z = nan;
    }

    auto js = std::make_unique<sensor_msgs::msg::JointState>();
    js->header = m->header;
    js->name = kJointNames;
    js->position.assign(q.begin(), q.end());
    js->velocity.assign(qd.begin(), qd.end());

    auto ps = std::make_unique<geometry_msgs::msg::PoseStamped>();
    ps->header = m->header;
    ps->pose = m->tcp_base;

    auto w = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    w->header = m->header;
    w->wrench.force.x = force[0];
    w->wrench.force.y = force[1];
    w->wrench.force.z = force[2];
    w->wrench.torque.x = force[3];
    w->wrench.torque.y = force[4];
    w->wrench.torque.z = force[5];

    pub_state_->publish(std::move(m));
    pub_js_->publish(std::move(js));
    pub_pose_->publish(std::move(ps));
    pub_wrench_->publish(std::move(w));
    if (nocmd) {
      pub_nocmd_->publish(std::move(nocmd));
    }
  }

  void log_stats()
  {
    long samples, repeated, gaps, lost;
    std::vector<double> d;
    {
      std::lock_guard<std::mutex> lk(stats_mtx_);
      samples = samples_;
      repeated = repeated_;
      gaps = gaps_;
      lost = lost_;
      d.swap(stamp_minus_ctrl_);
      samples_ = repeated_ = gaps_ = lost_ = 0;
    }
    double p50 = 0, p99 = 0;
    if (!d.empty()) {
      std::vector<double> s = d;
      std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
      const double med = s[s.size() / 2];
      for (auto& x : d) {
        x = std::abs(x - med) * 1e3;
      }
      std::sort(d.begin(), d.end());
      p50 = d[d.size() / 2];
      p99 = d[std::min(d.size() - 1, static_cast<size_t>(0.99 * d.size()))];
    }
    RCLCPP_INFO(get_logger(),
                "RTDE %ld Hz, repeated ctrl time %ld, gaps %ld (samples lost %ld), jitter(stamp-ctrl) "
                "p50=%.3f ms p99=%.3f ms, read failures %ld, control status %s, freedrive samples %ld",
                samples, repeated, gaps, lost, p50, p99, read_failures_.exchange(0),
                have_status_ ? "received" : "none (no robot_control)", freedrive_samples_.exchange(0));
  }

  std::string ip_, frame_;
  double freq_;
  int rt_priority_;
  urcl::comm::INotifier notifier_;
  std::unique_ptr<urcl::rtde_interface::RTDEClient> client_;
  rclcpp::Publisher<RobotState>::SharedPtr pub_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_nocmd_;
  int freedrive_input_ = -1;
  int64_t status_t_ns_ = 0;
  long nocmd_counter_ = 0;
  std::atomic<long> freedrive_samples_{0};
  rclcpp::Subscription<ControlStatus>::SharedPtr sub_status_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::thread reader_;
  std::atomic<bool> stop_{false};
  std::mutex status_mtx_, stats_mtx_;
  ControlStatus status_;
  std::atomic<bool> have_status_{false};
  Mat4 inv_phantom_{};
  bool have_phantom_ = false;
  long samples_ = 0, repeated_ = 0, gaps_ = 0, lost_ = 0;
  std::atomic<long> read_failures_{0};
  double last_ctrl_ = 0;
  bool have_last_ = false;
  std::vector<double> stamp_minus_ctrl_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<RobotStateCpp>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "[robot_state_cpp] %s\n", e.what());
    rc = 2;
  }
  rclcpp::shutdown();
  return rc;
}
