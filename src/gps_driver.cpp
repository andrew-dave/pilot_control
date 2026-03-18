/**
 * @file gps_driver.cpp
 * @brief ROS2 driver for u-blox ZED-F9P GNSS receiver
 *
 * Features:
 * - UBX protocol parsing (NAV-PVT plus raw-message health accounting)
 * - Quality gating with configurable thresholds
 * - Auto-reconnection on USB disconnect
 * - Receiver configuration on startup
 * - Session-oriented raw UBX logging for post-processing
 * - Dedicated raw GNSS health topic/service for operator visibility
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string jsonEscape(const std::string& input) {
  std::ostringstream escaped;
  for (const char c : input) {
    switch (c) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default: escaped << c; break;
    }
  }
  return escaped.str();
}

std::string formatWallTime(const std::chrono::system_clock::time_point& tp) {
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(tp);
  std::tm utc_tm{};
  gmtime_r(&raw_time, &utc_tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
  return std::string(buf);
}

std::string formatNowWallTime() {
  return formatWallTime(std::chrono::system_clock::now());
}

void appendUniqueReason(std::vector<std::string>& reasons, const std::string& reason) {
  if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end()) {
    reasons.push_back(reason);
  }
}

std::string joinReasons(const std::vector<std::string>& reasons) {
  std::ostringstream out;
  for (size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) {
      out << "; ";
    }
    out << reasons[i];
  }
  return out.str();
}

}  // namespace

// UBX protocol constants
constexpr uint8_t UBX_SYNC1 = 0xB5;
constexpr uint8_t UBX_SYNC2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_CLASS_RXM = 0x02;

constexpr uint8_t UBX_NAV_PVT = 0x07;
constexpr uint8_t UBX_NAV_HPPOSLLH = 0x14;
constexpr uint8_t UBX_NAV_SAT = 0x35;
constexpr uint8_t UBX_RXM_SFRBX = 0x13;
constexpr uint8_t UBX_RXM_RAWX = 0x15;

constexpr size_t UBX_MAX_PAYLOAD_LEN = 4096;
constexpr size_t UBX_BUFFER_OVERFLOW_LIMIT = 65536;

// Dynamic model values
constexpr uint8_t DYN_MODEL_PORTABLE = 0;
constexpr uint8_t DYN_MODEL_STATIONARY = 2;
constexpr uint8_t DYN_MODEL_PEDESTRIAN = 3;
constexpr uint8_t DYN_MODEL_AUTOMOTIVE = 4;
constexpr uint8_t DYN_MODEL_SEA = 5;
constexpr uint8_t DYN_MODEL_AIRBORNE_1G = 6;
constexpr uint8_t DYN_MODEL_AIRBORNE_2G = 7;
constexpr uint8_t DYN_MODEL_AIRBORNE_4G = 8;
constexpr uint8_t DYN_MODEL_WRIST = 9;
constexpr uint8_t DYN_MODEL_BIKE = 10;

// NAV-PVT payload structure (92 bytes)
#pragma pack(push, 1)
struct UbxNavPvt {
  uint32_t iTOW;        // GPS time of week (ms)
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fixType;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  uint8_t flags3;
  uint8_t reserved1[5];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
};
#pragma pack(pop)

static_assert(sizeof(UbxNavPvt) == 92, "UbxNavPvt size must be 92 bytes");

class GpsDriver : public rclcpp::Node {
public:
  GpsDriver() : Node("gps_driver"), fd_(-1), running_(false) {
    // Core configuration
    this->declare_parameter<std::string>("device", "/dev/gps");
    this->declare_parameter<int>("baud_rate", 38400);
    this->declare_parameter<std::string>("frame_id", "gps_link");

    // Quality gating
    this->declare_parameter<double>("hacc_max_m", 5.0);
    this->declare_parameter<int>("min_sats", 8);
    this->declare_parameter<double>("pdop_max", 4.0);
    this->declare_parameter<bool>("require_3d_fix", true);
    this->declare_parameter<bool>("require_gnss_fix_ok", true);

    // Receiver configuration
    this->declare_parameter<bool>("configure_receiver", true);
    this->declare_parameter<int>("rate_hz", 5);
    this->declare_parameter<int>("dynamic_model", DYN_MODEL_AUTOMOTIVE);
    this->declare_parameter<bool>("enable_raw_observation_messages", true);
    this->declare_parameter<bool>("enable_nav_sat", true);
    this->declare_parameter<bool>("enable_hpposllh", true);

    // Raw health thresholds
    this->declare_parameter<double>("rawx_startup_timeout_sec", 5.0);
    this->declare_parameter<double>("sfrbx_startup_timeout_sec", 20.0);
    this->declare_parameter<double>("rawx_min_coverage_ratio", 0.8);
    this->declare_parameter<double>("rawx_gap_warn_sec", 2.0);
    this->declare_parameter<double>("rawx_gap_fail_sec", 10.0);
    this->declare_parameter<double>("raw_log_flush_interval_sec", 1.0);
    this->declare_parameter<double>("raw_log_fsync_interval_sec", 5.0);

    // Raw logging
    this->declare_parameter<std::string>("raw_log_target_path", "");

    // Load parameters
    device_ = this->get_parameter("device").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    hacc_max_m_ = this->get_parameter("hacc_max_m").as_double();
    min_sats_ = this->get_parameter("min_sats").as_int();
    pdop_max_ = this->get_parameter("pdop_max").as_double();
    require_3d_fix_ = this->get_parameter("require_3d_fix").as_bool();
    require_gnss_fix_ok_ = this->get_parameter("require_gnss_fix_ok").as_bool();
    configure_receiver_ = this->get_parameter("configure_receiver").as_bool();
    rate_hz_ = this->get_parameter("rate_hz").as_int();
    dynamic_model_ = this->get_parameter("dynamic_model").as_int();
    enable_raw_observation_messages_ = this->get_parameter("enable_raw_observation_messages").as_bool();
    enable_nav_sat_ = this->get_parameter("enable_nav_sat").as_bool();
    enable_hpposllh_ = this->get_parameter("enable_hpposllh").as_bool();
    rawx_startup_timeout_sec_ = this->get_parameter("rawx_startup_timeout_sec").as_double();
    sfrbx_startup_timeout_sec_ = this->get_parameter("sfrbx_startup_timeout_sec").as_double();
    rawx_min_coverage_ratio_ = this->get_parameter("rawx_min_coverage_ratio").as_double();
    rawx_gap_warn_sec_ = this->get_parameter("rawx_gap_warn_sec").as_double();
    rawx_gap_fail_sec_ = this->get_parameter("rawx_gap_fail_sec").as_double();
    raw_log_flush_interval_sec_ = this->get_parameter("raw_log_flush_interval_sec").as_double();
    raw_log_fsync_interval_sec_ = this->get_parameter("raw_log_fsync_interval_sec").as_double();
    raw_log_target_path_ = this->get_parameter("raw_log_target_path").as_string();
    driver_boot_time_ = formatNowWallTime();

    fix_raw_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix_raw", 10);
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/gps/vel", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/gps/diag", 10);
    raw_health_pub_ = this->create_publisher<std_msgs::msg::String>("/gps/raw_health", 10);

    raw_log_set_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/gps/raw_log_set",
        std::bind(&GpsDriver::onRawLogSet, this, std::placeholders::_1, std::placeholders::_2));
    raw_log_promote_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/gps/raw_log_promote",
        std::bind(&GpsDriver::onRawLogPromote, this, std::placeholders::_1, std::placeholders::_2));
    raw_health_snapshot_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/gps/raw_health_snapshot",
        std::bind(&GpsDriver::onRawHealthSnapshot, this, std::placeholders::_1, std::placeholders::_2));

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GpsDriver::onParametersSet, this, std::placeholders::_1));

    health_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GpsDriver::publishRawHealth, this));

    running_ = true;
    serial_thread_ = std::thread(&GpsDriver::serialLoop, this);

    RCLCPP_INFO(this->get_logger(),
                "GPS Driver started: device=%s baud=%d rate=%dHz dynamic_model=%d",
                device_.c_str(), baud_rate_, rate_hz_, dynamic_model_);
    RCLCPP_INFO(this->get_logger(),
                "Quality gates: hAcc<%.1fm, numSV>=%d, pDOP<%.1f, 3D_fix=%s, gnss_fix_ok=%s",
                hacc_max_m_, min_sats_, pdop_max_,
                require_3d_fix_ ? "required" : "optional",
                require_gnss_fix_ok_ ? "required" : "optional");
    RCLCPP_INFO(this->get_logger(),
                "Raw PPK logging thresholds: RAWX startup %.1fs, SFRBX startup %.1fs, gap warn %.1fs, gap fail %.1fs",
                rawx_startup_timeout_sec_, sfrbx_startup_timeout_sec_, rawx_gap_warn_sec_, rawx_gap_fail_sec_);
    RCLCPP_INFO(this->get_logger(),
                "Raw UBX logging services ready: /gps/raw_log_set, /gps/raw_log_promote, /gps/raw_health_snapshot");
  }

  ~GpsDriver() override {
    running_ = false;
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lk(raw_log_mutex_);
      closeRawLogLocked();
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  struct RawHealthSnapshot {
    bool raw_log_active{false};
    bool rawx_seen{false};
    bool sfrbx_seen{false};
    bool gnss_invalid{false};
    bool rawx_startup_failed{false};
    bool sfrbx_startup_warned{false};
    bool rawx_gap_warned{false};
    bool rawx_gap_invalid{false};
    std::string raw_log_path;
    std::string raw_log_error;
    std::string driver_boot_time;
    std::string raw_log_start_time;
    std::string first_raw_obs_time;
    std::string last_raw_obs_time;
    std::string first_sfrbx_time;
    std::string last_sfrbx_time;
    uint64_t raw_log_bytes{0};
    uint64_t rawx_count{0};
    uint64_t sfrbx_count{0};
    uint64_t nav_sat_count{0};
    uint64_t hpposllh_count{0};
    uint64_t logging_failures{0};
    uint64_t flush_count{0};
    uint64_t fsync_count{0};
    uint64_t serial_reconnects{0};
    double raw_log_elapsed_sec{0.0};
    double current_rawx_gap_sec{0.0};
    double max_rawx_gap_sec{0.0};
    double rawx_expected_epochs{0.0};
    double rawx_coverage_ratio{0.0};
    std::vector<std::string> warning_reasons;
    std::vector<std::string> invalid_reasons;
    std::string health_level{"idle"};
  };

  rcl_interfaces::msg::SetParametersResult onParametersSet(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "accepted";

    for (const auto& param : parameters) {
      if (param.get_name() == "raw_log_target_path" &&
          param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        std::lock_guard<std::mutex> lk(raw_log_mutex_);
        raw_log_target_path_ = param.as_string();
      }
    }

    return result;
  }

  void onRawLogSet(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    const std::string target_path = this->get_parameter("raw_log_target_path").as_string();

    std::lock_guard<std::mutex> lk(raw_log_mutex_);

    if (req->data) {
      if (target_path.empty()) {
        raw_log_last_error_ = "raw_log_target_path is empty";
        resp->success = false;
        resp->message = raw_log_last_error_;
        return;
      }

      if (raw_logging_active_.load() && raw_log_stream_.is_open() &&
          current_raw_log_path_ == target_path) {
        resp->success = true;
        resp->message = "Raw UBX logging already active: " + current_raw_log_path_;
        return;
      }

      if (raw_log_stream_.is_open()) {
        closeRawLogLocked();
      }

      if (!openRawLogLocked(target_path, false, true)) {
        resp->success = false;
        resp->message = raw_log_last_error_;
        return;
      }

      resp->success = true;
      resp->message = "Raw UBX logging started: " + current_raw_log_path_;
      return;
    }

    const std::string previous_path = current_raw_log_path_;
    const bool was_active = raw_logging_active_.load() || raw_log_stream_.is_open();
    closeRawLogLocked();
    resp->success = true;
    resp->message = was_active ? "Raw UBX logging stopped: " + previous_path
                               : "Raw UBX logging already stopped";
  }

  void onRawLogPromote(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    const std::string target_path = this->get_parameter("raw_log_target_path").as_string();

    std::lock_guard<std::mutex> lk(raw_log_mutex_);

    if (target_path.empty()) {
      raw_log_last_error_ = "raw_log_target_path is empty";
      resp->success = false;
      resp->message = raw_log_last_error_;
      return;
    }

    if (current_raw_log_path_.empty()) {
      raw_log_last_error_ = "No raw UBX log exists to promote";
      resp->success = false;
      resp->message = raw_log_last_error_;
      return;
    }

    if (!promoteRawLogLocked(target_path)) {
      resp->success = false;
      resp->message = raw_log_last_error_;
      return;
    }

    resp->success = true;
    resp->message = "Raw UBX log promoted to: " + current_raw_log_path_;
  }

  void onRawHealthSnapshot(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    const RawHealthSnapshot snapshot = getRawHealthSnapshot();
    resp->success = true;
    resp->message = buildRawHealthJson(snapshot);
  }

  bool openSerial() {
    std::vector<std::string> devices = {device_, "/dev/ttyACM0"};

    for (const auto& dev : devices) {
      fd_ = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd_ >= 0) {
        if (dev != device_) {
          RCLCPP_WARN(this->get_logger(), "Primary device %s unavailable, using %s",
                      device_.c_str(), dev.c_str());
        }
        break;
      }
    }

    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Cannot open GPS device: %s", strerror(errno));
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    speed_t baud;
    switch (baud_rate_) {
      case 460800: baud = B460800; break;
      case 230400: baud = B230400; break;
      case 115200: baud = B115200; break;
      case 57600: baud = B57600; break;
      case 38400: baud = B38400; break;
      case 19200: baud = B19200; break;
      default: baud = B9600; break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    if (serial_open_count_ > 0) {
      serial_reconnects_++;
    }
    serial_open_count_++;

    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud",
                device_.c_str(), baud_rate_);

    if (configure_receiver_) {
      configureReceiver();
    }

    return true;
  }

  void configureReceiver() {
    RCLCPP_INFO(this->get_logger(), "Configuring ZED-F9P receiver...");

    usleep(100000);

    const uint16_t rate_ms = static_cast<uint16_t>(1000 / std::max(rate_hz_, 1));
    uint8_t set_rate[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x08,
      0x06, 0x00,
      static_cast<uint8_t>(rate_ms & 0xFF),
      static_cast<uint8_t>((rate_ms >> 8) & 0xFF),
      0x01, 0x00,
      0x01, 0x00,
      0x00, 0x00
    };
    sendUbxMessage(set_rate, sizeof(set_rate));

    uint8_t set_dyn[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x8A,
      0x09, 0x00,
      0x00,
      0x01,
      0x00, 0x00,
      0x21, 0x00, 0x11, 0x20,
      static_cast<uint8_t>(dynamic_model_),
      0x00, 0x00
    };
    sendUbxMessage(set_dyn, sizeof(set_dyn));

    setMessageRate(UBX_CLASS_NAV, UBX_NAV_PVT, 0x01);
    if (enable_raw_observation_messages_) {
      setMessageRate(UBX_CLASS_RXM, UBX_RXM_RAWX, 0x01);
      setMessageRate(UBX_CLASS_RXM, UBX_RXM_SFRBX, 0x01);
    }
    if (enable_nav_sat_) {
      setMessageRate(UBX_CLASS_NAV, UBX_NAV_SAT, 0x01);
    }
    if (enable_hpposllh_) {
      setMessageRate(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 0x01);
    }

    const uint8_t nmea_msgs[][2] = {
      {0xF0, 0x00},
      {0xF0, 0x01},
      {0xF0, 0x02},
      {0xF0, 0x03},
      {0xF0, 0x04},
      {0xF0, 0x05},
    };
    for (const auto& msg : nmea_msgs) {
      setMessageRate(msg[0], msg[1], 0x00);
    }

    usleep(200000);

    std::vector<std::string> enabled_messages = {"UBX-NAV-PVT"};
    if (enable_raw_observation_messages_) {
      enabled_messages.emplace_back("UBX-RXM-RAWX");
      enabled_messages.emplace_back("UBX-RXM-SFRBX");
    }
    if (enable_nav_sat_) {
      enabled_messages.emplace_back("UBX-NAV-SAT");
    }
    if (enable_hpposllh_) {
      enabled_messages.emplace_back("UBX-NAV-HPPOSLLH");
    }

    std::ostringstream msg_list;
    for (size_t i = 0; i < enabled_messages.size(); ++i) {
      if (i > 0) {
        msg_list << ", ";
      }
      msg_list << enabled_messages[i];
    }

    RCLCPP_INFO(this->get_logger(),
                "Receiver configured: rate=%dHz, dynamic_model=%d, messages=%s",
                rate_hz_, dynamic_model_, msg_list.str().c_str());
  }

  void setMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    uint8_t cfg_msg[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x01,
      0x03, 0x00,
      msg_class, msg_id,
      rate,
      0x00, 0x00
    };
    sendUbxMessage(cfg_msg, sizeof(cfg_msg));
  }

  void sendUbxMessage(uint8_t* msg, size_t len) {
    if (fd_ < 0) {
      return;
    }

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < len - 2; ++i) {
      ck_a += msg[i];
      ck_b += ck_a;
    }
    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;

    const ssize_t written = write(fd_, msg, len);
    if (written != static_cast<ssize_t>(len)) {
      RCLCPP_WARN(this->get_logger(), "UBX write incomplete: %zd/%zu bytes", written, len);
    }
    usleep(50000);
  }

  bool verifyChecksum(const std::vector<uint8_t>& frame) const {
    if (frame.size() < 8) {
      return false;
    }
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < frame.size() - 2; ++i) {
      ck_a += frame[i];
      ck_b += ck_a;
    }
    return frame[frame.size() - 2] == ck_a && frame[frame.size() - 1] == ck_b;
  }

  void resetRawHealthStateLocked(const std::chrono::steady_clock::time_point& now) {
    raw_log_start_steady_ = now;
    raw_log_start_wall_time_ = std::chrono::system_clock::now();
    raw_log_start_iso_ = formatWallTime(raw_log_start_wall_time_.value());

    last_flush_steady_ = now;
    last_fsync_steady_ = now;

    rawx_count_ = 0;
    sfrbx_count_ = 0;
    nav_sat_count_ = 0;
    hpposllh_count_ = 0;
    logging_failures_ = 0;
    flush_count_ = 0;
    fsync_count_ = 0;
    max_rawx_gap_sec_ = 0.0;

    rawx_first_steady_.reset();
    rawx_last_steady_.reset();
    sfrbx_first_steady_.reset();
    sfrbx_last_steady_.reset();

    rawx_first_iso_.clear();
    rawx_last_iso_.clear();
    sfrbx_first_iso_.clear();
    sfrbx_last_iso_.clear();

    rawx_startup_failed_ = false;
    sfrbx_startup_warned_ = false;
    rawx_gap_warned_ = false;
    rawx_gap_invalid_ = false;
    gnss_invalid_ = false;

    warning_reasons_.clear();
    invalid_reasons_.clear();
  }

  void markWarningLocked(const std::string& reason) {
    appendUniqueReason(warning_reasons_, reason);
  }

  void markInvalidLocked(const std::string& reason) {
    gnss_invalid_ = true;
    appendUniqueReason(invalid_reasons_, reason);
  }

  void recordLoggingFailureLocked(const std::string& reason, bool stop_logging) {
    ++logging_failures_;
    raw_log_last_error_ = reason;
    markInvalidLocked(reason);
    if (stop_logging) {
      if (raw_log_stream_.is_open()) {
        raw_log_stream_.close();
      }
      if (raw_log_sync_fd_ >= 0) {
        close(raw_log_sync_fd_);
        raw_log_sync_fd_ = -1;
      }
      raw_logging_active_.store(false);
    }
  }

  bool openRawLogLocked(const std::string& target_path, bool append, bool reset_health_state) {
    if (target_path.empty()) {
      raw_log_last_error_ = "raw_log_target_path is empty";
      return false;
    }

    fs::path path(target_path);
    std::error_code ec;
    if (path.has_parent_path()) {
      fs::create_directories(path.parent_path(), ec);
      if (ec) {
        raw_log_last_error_ = "Failed to create raw log directory: " + ec.message();
        return false;
      }
    }

    std::ios::openmode mode = std::ios::out | std::ios::binary;
    mode |= append ? std::ios::app : std::ios::trunc;
    raw_log_stream_.clear();
    raw_log_stream_.open(path, mode);
    if (!raw_log_stream_.is_open()) {
      raw_log_last_error_ = "Failed to open raw UBX log: " + target_path;
      return false;
    }

    raw_log_sync_fd_ = open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (raw_log_sync_fd_ < 0) {
      raw_log_stream_.close();
      raw_log_last_error_ = "Failed to open raw UBX sync handle: " + std::string(strerror(errno));
      return false;
    }

    current_raw_log_path_ = target_path;
    raw_logging_active_.store(true);

    if (append) {
      std::error_code size_ec;
      const auto size = fs::file_size(path, size_ec);
      raw_log_bytes_written_.store(size_ec ? 0 : size);
    } else {
      raw_log_bytes_written_.store(0);
    }

    const auto now = std::chrono::steady_clock::now();
    if (reset_health_state || !raw_log_start_steady_.has_value()) {
      resetRawHealthStateLocked(now);
    } else {
      last_flush_steady_ = now;
      last_fsync_steady_ = now;
    }

    raw_log_last_error_.clear();
    return true;
  }

  void closeRawLogLocked() {
    if (raw_log_stream_.is_open()) {
      raw_log_stream_.flush();
      if (raw_log_sync_fd_ >= 0) {
        fsync(raw_log_sync_fd_);
      }
      raw_log_stream_.close();
    }
    if (raw_log_sync_fd_ >= 0) {
      close(raw_log_sync_fd_);
      raw_log_sync_fd_ = -1;
    }
    raw_logging_active_.store(false);
  }

  bool promoteRawLogLocked(const std::string& target_path) {
    if (target_path.empty()) {
      raw_log_last_error_ = "raw_log_target_path is empty";
      return false;
    }

    if (current_raw_log_path_.empty()) {
      raw_log_last_error_ = "No active raw UBX log path to promote";
      return false;
    }

    const fs::path source(current_raw_log_path_);
    const fs::path target(target_path);

    if (!fs::exists(source)) {
      raw_log_last_error_ = "Raw UBX source file does not exist: " + source.string();
      return false;
    }

    if (source == target) {
      if (!raw_log_stream_.is_open()) {
        return openRawLogLocked(target.string(), true, false);
      }
      return true;
    }

    const bool was_active = raw_log_stream_.is_open();
    closeRawLogLocked();

    std::error_code ec;
    if (target.has_parent_path()) {
      fs::create_directories(target.parent_path(), ec);
      if (ec) {
        raw_log_last_error_ = "Failed to create promoted raw log directory: " + ec.message();
        return false;
      }
    }

    if (fs::exists(target)) {
      raw_log_last_error_ = "Promote destination already exists: " + target.string();
      return false;
    }

    fs::rename(source, target, ec);
    if (ec) {
      ec.clear();
      fs::copy_file(source, target, fs::copy_options::none, ec);
      if (ec) {
        raw_log_last_error_ = "Failed to move raw UBX log to session folder: " + ec.message();
        return false;
      }
      std::error_code remove_ec;
      fs::remove(source, remove_ec);
    }

    current_raw_log_path_ = target.string();

    if (was_active && !openRawLogLocked(current_raw_log_path_, true, false)) {
      return false;
    }

    if (!was_active) {
      std::error_code size_ec;
      const auto size = fs::file_size(target, size_ec);
      raw_log_bytes_written_.store(size_ec ? 0 : size);
      raw_log_last_error_.clear();
    }

    return true;
  }

  void updateRawLogMaintenanceLocked(const std::chrono::steady_clock::time_point& now) {
    if (!raw_log_stream_.is_open()) {
      return;
    }

    if (!last_flush_steady_.has_value()) {
      last_flush_steady_ = now;
    }
    if (!last_fsync_steady_.has_value()) {
      last_fsync_steady_ = now;
    }

    const double since_flush = std::chrono::duration<double>(now - *last_flush_steady_).count();
    if (since_flush >= raw_log_flush_interval_sec_) {
      raw_log_stream_.flush();
      if (!raw_log_stream_) {
        recordLoggingFailureLocked("Failed to flush raw UBX log", true);
        RCLCPP_ERROR(this->get_logger(), "%s", raw_log_last_error_.c_str());
        return;
      }
      ++flush_count_;
      last_flush_steady_ = now;
    }

    const double since_fsync = std::chrono::duration<double>(now - *last_fsync_steady_).count();
    if (raw_log_sync_fd_ >= 0 && since_fsync >= raw_log_fsync_interval_sec_) {
      raw_log_stream_.flush();
      if (!raw_log_stream_) {
        recordLoggingFailureLocked("Failed to flush raw UBX log before fsync", true);
        RCLCPP_ERROR(this->get_logger(), "%s", raw_log_last_error_.c_str());
        return;
      }
      if (fsync(raw_log_sync_fd_) != 0) {
        const std::string error = "Failed to fsync raw UBX log: " + std::string(strerror(errno));
        recordLoggingFailureLocked(error, false);
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", error.c_str());
      } else {
        ++fsync_count_;
        last_fsync_steady_ = now;
      }
    }
  }

  void observeVerifiedFrame(uint8_t msg_class, uint8_t msg_id) {
    const auto now_steady = std::chrono::steady_clock::now();
    const std::string now_iso = formatNowWallTime();

    std::lock_guard<std::mutex> lk(raw_log_mutex_);
    switch (msg_class) {
      case UBX_CLASS_RXM:
        if (msg_id == UBX_RXM_RAWX) {
          ++rawx_count_;
          if (!rawx_first_steady_.has_value()) {
            rawx_first_steady_ = now_steady;
            rawx_first_iso_ = now_iso;
          }
          if (rawx_last_steady_.has_value()) {
            const double gap_sec =
                std::chrono::duration<double>(now_steady - *rawx_last_steady_).count();
            max_rawx_gap_sec_ = std::max(max_rawx_gap_sec_, gap_sec);
          }
          rawx_last_steady_ = now_steady;
          rawx_last_iso_ = now_iso;
        } else if (msg_id == UBX_RXM_SFRBX) {
          ++sfrbx_count_;
          if (!sfrbx_first_steady_.has_value()) {
            sfrbx_first_steady_ = now_steady;
            sfrbx_first_iso_ = now_iso;
          }
          sfrbx_last_steady_ = now_steady;
          sfrbx_last_iso_ = now_iso;
        }
        break;
      case UBX_CLASS_NAV:
        if (msg_id == UBX_NAV_SAT) {
          ++nav_sat_count_;
        } else if (msg_id == UBX_NAV_HPPOSLLH) {
          ++hpposllh_count_;
        }
        break;
      default:
        break;
    }
  }

  void evaluateRawHealthLocked(const std::chrono::steady_clock::time_point& now) {
    if (!raw_log_start_steady_.has_value()) {
      return;
    }

    const double elapsed_sec = std::chrono::duration<double>(now - *raw_log_start_steady_).count();

    if (rawx_count_ == 0 && elapsed_sec >= rawx_startup_timeout_sec_ && !rawx_startup_failed_) {
      rawx_startup_failed_ = true;
      const std::string reason =
          "RAWX missing after startup window (" + std::to_string(rawx_startup_timeout_sec_) + " s)";
      markInvalidLocked(reason);
      RCLCPP_ERROR(this->get_logger(),
                   "PPK raw observation health failed: %s", reason.c_str());
    }

    if (sfrbx_count_ == 0 && elapsed_sec >= sfrbx_startup_timeout_sec_ && !sfrbx_startup_warned_) {
      sfrbx_startup_warned_ = true;
      const std::string reason =
          "SFRBX missing after startup window (" + std::to_string(sfrbx_startup_timeout_sec_) + " s)";
      markWarningLocked(reason);
      RCLCPP_WARN(this->get_logger(),
                  "PPK raw navigation warning: %s", reason.c_str());
    }

    if (raw_logging_active_.load() && rawx_last_steady_.has_value()) {
      const double current_gap_sec =
          std::chrono::duration<double>(now - *rawx_last_steady_).count();
      max_rawx_gap_sec_ = std::max(max_rawx_gap_sec_, current_gap_sec);

      if (current_gap_sec > rawx_gap_warn_sec_ && !rawx_gap_warned_) {
        rawx_gap_warned_ = true;
        const std::string reason =
            "RAWX gap exceeded warning threshold (" + std::to_string(rawx_gap_warn_sec_) + " s)";
        markWarningLocked(reason);
        RCLCPP_WARN(this->get_logger(), "PPK raw observation warning: %s", reason.c_str());
      }

      if (current_gap_sec > rawx_gap_fail_sec_ && !rawx_gap_invalid_) {
        rawx_gap_invalid_ = true;
        const std::string reason =
            "RAWX gap exceeded failure threshold (" + std::to_string(rawx_gap_fail_sec_) + " s)";
        markInvalidLocked(reason);
        RCLCPP_ERROR(this->get_logger(), "PPK raw observation invalidated: %s", reason.c_str());
      }
    }
  }

  RawHealthSnapshot getRawHealthSnapshot() {
    std::lock_guard<std::mutex> lk(raw_log_mutex_);
    const auto now = std::chrono::steady_clock::now();
    evaluateRawHealthLocked(now);

    RawHealthSnapshot snapshot;
    snapshot.raw_log_active = raw_logging_active_.load();
    snapshot.rawx_seen = rawx_count_ > 0;
    snapshot.sfrbx_seen = sfrbx_count_ > 0;
    snapshot.gnss_invalid = gnss_invalid_;
    snapshot.rawx_startup_failed = rawx_startup_failed_;
    snapshot.sfrbx_startup_warned = sfrbx_startup_warned_;
    snapshot.rawx_gap_warned = rawx_gap_warned_;
    snapshot.rawx_gap_invalid = rawx_gap_invalid_;
    snapshot.raw_log_path = current_raw_log_path_.empty() ? raw_log_target_path_ : current_raw_log_path_;
    snapshot.raw_log_error = raw_log_last_error_;
    snapshot.driver_boot_time = driver_boot_time_;
    snapshot.raw_log_start_time = raw_log_start_iso_;
    snapshot.first_raw_obs_time = rawx_first_iso_;
    snapshot.last_raw_obs_time = rawx_last_iso_;
    snapshot.first_sfrbx_time = sfrbx_first_iso_;
    snapshot.last_sfrbx_time = sfrbx_last_iso_;
    snapshot.raw_log_bytes = raw_log_bytes_written_.load();
    snapshot.rawx_count = rawx_count_;
    snapshot.sfrbx_count = sfrbx_count_;
    snapshot.nav_sat_count = nav_sat_count_;
    snapshot.hpposllh_count = hpposllh_count_;
    snapshot.logging_failures = logging_failures_;
    snapshot.flush_count = flush_count_;
    snapshot.fsync_count = fsync_count_;
    snapshot.serial_reconnects = serial_reconnects_.load();
    snapshot.warning_reasons = warning_reasons_;
    snapshot.invalid_reasons = invalid_reasons_;
    snapshot.max_rawx_gap_sec = max_rawx_gap_sec_;

    if (raw_log_start_steady_.has_value()) {
      snapshot.raw_log_elapsed_sec =
          std::chrono::duration<double>(now - *raw_log_start_steady_).count();
      snapshot.rawx_expected_epochs = snapshot.raw_log_elapsed_sec * static_cast<double>(std::max(rate_hz_, 1));
      if (snapshot.rawx_expected_epochs > 0.0) {
        snapshot.rawx_coverage_ratio =
            static_cast<double>(snapshot.rawx_count) / snapshot.rawx_expected_epochs;
      }
    }
    if (rawx_last_steady_.has_value()) {
      snapshot.current_rawx_gap_sec =
          std::chrono::duration<double>(now - *rawx_last_steady_).count();
    }

    if (snapshot.gnss_invalid) {
      snapshot.health_level = "error";
    } else if (!snapshot.warning_reasons.empty()) {
      snapshot.health_level = "warn";
    } else if (snapshot.raw_log_active || !snapshot.raw_log_start_time.empty()) {
      snapshot.health_level = "ok";
    } else {
      snapshot.health_level = "idle";
    }

    return snapshot;
  }

  std::string buildRawHealthJson(const RawHealthSnapshot& snapshot) const {
    auto quoted_or_null = [](const std::string& value) {
      return value.empty() ? std::string("null") : "\"" + jsonEscape(value) + "\"";
    };

    auto reasons_json = [](const std::vector<std::string>& reasons) {
      std::ostringstream out;
      out << "[";
      for (size_t i = 0; i < reasons.size(); ++i) {
        if (i > 0) {
          out << ",";
        }
        out << "\"" << jsonEscape(reasons[i]) << "\"";
      }
      out << "]";
      return out.str();
    };

    std::ostringstream json;
    json << "{"
         << "\"driver_boot_time\":" << quoted_or_null(snapshot.driver_boot_time) << ","
         << "\"health_level\":\"" << snapshot.health_level << "\","
         << "\"gnss_invalid\":" << (snapshot.gnss_invalid ? "true" : "false") << ","
         << "\"raw_log_active\":" << (snapshot.raw_log_active ? "true" : "false") << ","
         << "\"raw_log_path\":" << quoted_or_null(snapshot.raw_log_path) << ","
         << "\"raw_log_error\":" << quoted_or_null(snapshot.raw_log_error) << ","
         << "\"raw_log_start_time\":" << quoted_or_null(snapshot.raw_log_start_time) << ","
         << "\"raw_log_bytes\":" << snapshot.raw_log_bytes << ","
         << "\"logging_failures\":" << snapshot.logging_failures << ","
         << "\"serial_reconnects\":" << snapshot.serial_reconnects << ","
         << "\"flush_count\":" << snapshot.flush_count << ","
         << "\"fsync_count\":" << snapshot.fsync_count << ","
         << "\"rawx_seen\":" << (snapshot.rawx_seen ? "true" : "false") << ","
         << "\"sfrbx_seen\":" << (snapshot.sfrbx_seen ? "true" : "false") << ","
         << "\"rawx_count\":" << snapshot.rawx_count << ","
         << "\"sfrbx_count\":" << snapshot.sfrbx_count << ","
         << "\"nav_sat_count\":" << snapshot.nav_sat_count << ","
         << "\"hpposllh_count\":" << snapshot.hpposllh_count << ","
         << "\"first_raw_obs_time\":" << quoted_or_null(snapshot.first_raw_obs_time) << ","
         << "\"last_raw_obs_time\":" << quoted_or_null(snapshot.last_raw_obs_time) << ","
         << "\"first_sfrbx_time\":" << quoted_or_null(snapshot.first_sfrbx_time) << ","
         << "\"last_sfrbx_time\":" << quoted_or_null(snapshot.last_sfrbx_time) << ","
         << "\"raw_log_elapsed_sec\":" << snapshot.raw_log_elapsed_sec << ","
         << "\"rawx_expected_epochs\":" << snapshot.rawx_expected_epochs << ","
         << "\"rawx_coverage_ratio\":" << snapshot.rawx_coverage_ratio << ","
         << "\"current_rawx_gap_sec\":" << snapshot.current_rawx_gap_sec << ","
         << "\"max_rawx_gap_sec\":" << snapshot.max_rawx_gap_sec << ","
         << "\"rawx_startup_timeout_sec\":" << rawx_startup_timeout_sec_ << ","
         << "\"sfrbx_startup_timeout_sec\":" << sfrbx_startup_timeout_sec_ << ","
         << "\"rawx_min_coverage_ratio\":" << rawx_min_coverage_ratio_ << ","
         << "\"rawx_gap_warn_sec\":" << rawx_gap_warn_sec_ << ","
         << "\"rawx_gap_fail_sec\":" << rawx_gap_fail_sec_ << ","
         << "\"rawx_startup_failed\":" << (snapshot.rawx_startup_failed ? "true" : "false") << ","
         << "\"sfrbx_startup_warned\":" << (snapshot.sfrbx_startup_warned ? "true" : "false") << ","
         << "\"rawx_gap_warned\":" << (snapshot.rawx_gap_warned ? "true" : "false") << ","
         << "\"rawx_gap_invalid\":" << (snapshot.rawx_gap_invalid ? "true" : "false") << ","
         << "\"warning_reasons\":" << reasons_json(snapshot.warning_reasons) << ","
         << "\"invalid_reasons\":" << reasons_json(snapshot.invalid_reasons)
         << "}";
    return json.str();
  }

  void publishRawHealth() {
    const RawHealthSnapshot snapshot = getRawHealthSnapshot();
    std_msgs::msg::String msg;
    msg.data = buildRawHealthJson(snapshot);
    raw_health_pub_->publish(msg);
  }

  void writeRawBytes(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lk(raw_log_mutex_);
    if (!raw_logging_active_.load() || !raw_log_stream_.is_open()) {
      return;
    }

    raw_log_stream_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(len));
    if (!raw_log_stream_) {
      recordLoggingFailureLocked("Failed to write raw UBX bytes to log", true);
      RCLCPP_ERROR(this->get_logger(), "%s", raw_log_last_error_.c_str());
      return;
    }

    raw_log_bytes_written_.fetch_add(len);
    updateRawLogMaintenanceLocked(std::chrono::steady_clock::now());
  }

  void serialLoop() {
    std::vector<uint8_t> buffer;
    buffer.reserve(16384);

    while (running_) {
      if (fd_ < 0) {
        if (!openSerial()) {
          std::this_thread::sleep_for(std::chrono::seconds(2));
          continue;
        }
      }

      uint8_t tmp[2048];
      const ssize_t n = read(fd_, tmp, sizeof(tmp));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
          close(fd_);
          fd_ = -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      writeRawBytes(tmp, static_cast<size_t>(n));
      buffer.insert(buffer.end(), tmp, tmp + n);

      while (buffer.size() >= 8) {
        auto it = std::find(buffer.begin(), buffer.end(), UBX_SYNC1);
        if (it == buffer.end()) {
          buffer.clear();
          break;
        }
        if (it != buffer.begin()) {
          buffer.erase(buffer.begin(), it);
        }

        if (buffer.size() < 2 || buffer[1] != UBX_SYNC2) {
          buffer.erase(buffer.begin());
          continue;
        }

        if (buffer.size() < 6) {
          break;
        }

        const uint16_t payload_len = static_cast<uint16_t>(buffer[4] | (buffer[5] << 8));
        if (payload_len > UBX_MAX_PAYLOAD_LEN) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "Discarding oversized UBX payload: %u bytes", payload_len);
          buffer.erase(buffer.begin());
          continue;
        }

        const size_t frame_len = 6 + payload_len + 2;
        if (buffer.size() < frame_len) {
          break;
        }

        std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_len);
        buffer.erase(buffer.begin(), buffer.begin() + frame_len);

        if (!verifyChecksum(frame)) {
          checksum_errors_++;
          RCLCPP_DEBUG(this->get_logger(), "UBX checksum failed");
          continue;
        }

        observeVerifiedFrame(frame[2], frame[3]);

        if (frame[2] == UBX_CLASS_NAV && frame[3] == UBX_NAV_PVT &&
            payload_len >= sizeof(UbxNavPvt)) {
          processNavPvt(reinterpret_cast<const UbxNavPvt*>(&frame[6]));
        }
      }

      if (buffer.size() > UBX_BUFFER_OVERFLOW_LIMIT) {
        RCLCPP_WARN(this->get_logger(), "UBX buffer overflow (%zu bytes), clearing", buffer.size());
        buffer.clear();
      }
    }
  }

  void processNavPvt(const UbxNavPvt* pvt) {
    const auto now = this->get_clock()->now();
    msg_count_++;

    const double h_acc_m = pvt->hAcc * 1e-3;
    const double v_acc_m = pvt->vAcc * 1e-3;
    const double pdop = pvt->pDOP * 0.01;
    const bool gnss_fix_ok = (pvt->flags & 0x01) != 0;
    const bool diff_soln = (pvt->flags & 0x02) != 0;
    const uint8_t carr_soln = (pvt->flags >> 6) & 0x03;

    auto fix_msg = sensor_msgs::msg::NavSatFix();
    fix_msg.header.stamp = now;
    fix_msg.header.frame_id = frame_id_;
    fix_msg.latitude = pvt->lat * 1e-7;
    fix_msg.longitude = pvt->lon * 1e-7;
    fix_msg.altitude = pvt->hMSL * 1e-3;

    fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_msg.position_covariance[0] = h_acc_m * h_acc_m;
    fix_msg.position_covariance[4] = h_acc_m * h_acc_m;
    fix_msg.position_covariance[8] = v_acc_m * v_acc_m;

    if (pvt->fixType < 2) {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (diff_soln || carr_soln > 0) {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    fix_raw_pub_->publish(fix_msg);

    bool passes_gate = true;
    std::string reject_reason;

    if (require_3d_fix_ && pvt->fixType < 3) {
      passes_gate = false;
      reject_reason = "no 3D fix";
    } else if (require_gnss_fix_ok_ && !gnss_fix_ok) {
      passes_gate = false;
      reject_reason = "gnssFixOK=false";
    } else if (h_acc_m > hacc_max_m_) {
      passes_gate = false;
      reject_reason = "hAcc>" + std::to_string(hacc_max_m_);
    } else if (static_cast<int>(pvt->numSV) < min_sats_) {
      passes_gate = false;
      reject_reason = "numSV<" + std::to_string(min_sats_);
    } else if (pdop > pdop_max_) {
      passes_gate = false;
      reject_reason = "pDOP>" + std::to_string(pdop_max_);
    }

    if (passes_gate) {
      fix_pub_->publish(fix_msg);
      good_fixes_++;
    } else {
      rejected_fixes_++;
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "GPS fix rejected: %s", reject_reason.c_str());
    }

    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header = fix_msg.header;
    vel_msg.twist.linear.x = pvt->velE * 1e-3;
    vel_msg.twist.linear.y = pvt->velN * 1e-3;
    vel_msg.twist.linear.z = -pvt->velD * 1e-3;
    vel_pub_->publish(vel_msg);

    publishDiagnostics(now, pvt, h_acc_m, v_acc_m, pdop, gnss_fix_ok,
                       diff_soln, carr_soln, passes_gate);
  }

  void publishDiagnostics(const rclcpp::Time& now, const UbxNavPvt* pvt,
                          double h_acc_m, double v_acc_m, double pdop,
                          bool gnss_fix_ok, bool diff_soln, uint8_t carr_soln,
                          bool quality_ok) {
    static auto last_diag_time = std::chrono::steady_clock::now();
    const auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_diag_time).count() < 1000) {
      return;
    }
    last_diag_time = current_time;

    const RawHealthSnapshot raw_snapshot = getRawHealthSnapshot();

    auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
    diag_msg.header.stamp = now;

    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = "GPS - ZED-F9P";
    ds.hardware_id = "ublox_zed_f9p";

    if (quality_ok) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      ds.message = "GPS fix OK - publishing to /gps/fix";
    } else if (pvt->fixType >= 2) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      ds.message = "GPS fix below quality threshold - only /gps/fix_raw";
    } else {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      ds.message = "No GPS fix";
    }

    if (raw_snapshot.gnss_invalid) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      ds.message += "; raw GNSS invalid for PPK";
    } else if (!raw_snapshot.warning_reasons.empty() &&
               ds.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      ds.message += "; raw GNSS warnings present";
    }

    auto add_kv = [&](const std::string& key, const std::string& val) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = val;
      ds.values.push_back(kv);
    };

    add_kv("fix_type", std::to_string(pvt->fixType) +
                           (pvt->fixType == 0 ? " (none)" :
                            pvt->fixType == 2 ? " (2D)" :
                            pvt->fixType == 3 ? " (3D)" :
                            pvt->fixType == 4 ? " (3D+DR)" : ""));
    add_kv("num_satellites", std::to_string(pvt->numSV));
    add_kv("gnss_fix_ok", gnss_fix_ok ? "true" : "false");
    add_kv("differential", diff_soln ? "true" : "false");
    add_kv("carrier_solution", std::to_string(carr_soln) +
                                    (carr_soln == 0 ? " (none)" :
                                     carr_soln == 1 ? " (float)" :
                                     carr_soln == 2 ? " (fixed)" : ""));

    char buf[128];
    snprintf(buf, sizeof(buf), "%.2f m", h_acc_m);
    add_kv("h_accuracy", buf);
    snprintf(buf, sizeof(buf), "%.2f m", v_acc_m);
    add_kv("v_accuracy", buf);
    snprintf(buf, sizeof(buf), "%.2f", pdop);
    add_kv("pdop", buf);

    snprintf(buf, sizeof(buf), "%.7f", pvt->lat * 1e-7);
    add_kv("latitude", buf);
    snprintf(buf, sizeof(buf), "%.7f", pvt->lon * 1e-7);
    add_kv("longitude", buf);
    snprintf(buf, sizeof(buf), "%.2f m", pvt->hMSL * 1e-3);
    add_kv("altitude_msl", buf);

    snprintf(buf, sizeof(buf), "%.2f m/s", pvt->gSpeed * 1e-3);
    add_kv("ground_speed", buf);
    snprintf(buf, sizeof(buf), "%.1f deg", pvt->headMot * 1e-5);
    add_kv("heading", buf);

    add_kv("messages_received", std::to_string(msg_count_.load()));
    add_kv("good_fixes", std::to_string(good_fixes_.load()));
    add_kv("rejected_fixes", std::to_string(rejected_fixes_.load()));
    add_kv("checksum_errors", std::to_string(checksum_errors_.load()));
    add_kv("serial_reconnects", std::to_string(serial_reconnects_.load()));

    const double accept_rate = msg_count_ > 0 ? 100.0 * good_fixes_ / msg_count_ : 0.0;
    snprintf(buf, sizeof(buf), "%.1f%%", accept_rate);
    add_kv("acceptance_rate", buf);

    add_kv("raw_health_level", raw_snapshot.health_level);
    add_kv("raw_log_active", raw_snapshot.raw_log_active ? "true" : "false");
    add_kv("raw_log_path", raw_snapshot.raw_log_path);
    add_kv("raw_log_bytes", std::to_string(raw_snapshot.raw_log_bytes));
    add_kv("raw_log_elapsed_sec", std::to_string(raw_snapshot.raw_log_elapsed_sec));
    add_kv("rawx_seen", raw_snapshot.rawx_seen ? "true" : "false");
    add_kv("rawx_count", std::to_string(raw_snapshot.rawx_count));
    add_kv("sfrbx_seen", raw_snapshot.sfrbx_seen ? "true" : "false");
    add_kv("sfrbx_count", std::to_string(raw_snapshot.sfrbx_count));
    add_kv("rawx_coverage_ratio", std::to_string(raw_snapshot.rawx_coverage_ratio));
    add_kv("current_rawx_gap_sec", std::to_string(raw_snapshot.current_rawx_gap_sec));
    add_kv("max_rawx_gap_sec", std::to_string(raw_snapshot.max_rawx_gap_sec));
    add_kv("gnss_invalid", raw_snapshot.gnss_invalid ? "true" : "false");
    add_kv("logging_failures", std::to_string(raw_snapshot.logging_failures));
    if (!raw_snapshot.raw_log_error.empty()) {
      add_kv("raw_log_error", raw_snapshot.raw_log_error);
    }
    if (!raw_snapshot.warning_reasons.empty()) {
      add_kv("raw_warning_reasons", joinReasons(raw_snapshot.warning_reasons));
    }
    if (!raw_snapshot.invalid_reasons.empty()) {
      add_kv("raw_invalid_reasons", joinReasons(raw_snapshot.invalid_reasons));
    }

    diag_msg.status.push_back(ds);
    diag_pub_->publish(diag_msg);
  }

  // Parameters
  std::string device_;
  int baud_rate_;
  std::string frame_id_;
  double hacc_max_m_;
  int min_sats_;
  double pdop_max_;
  bool require_3d_fix_;
  bool require_gnss_fix_ok_;
  bool configure_receiver_;
  int rate_hz_;
  int dynamic_model_;
  bool enable_raw_observation_messages_;
  bool enable_nav_sat_;
  bool enable_hpposllh_;
  double rawx_startup_timeout_sec_;
  double sfrbx_startup_timeout_sec_;
  double rawx_min_coverage_ratio_;
  double rawx_gap_warn_sec_;
  double rawx_gap_fail_sec_;
  double raw_log_flush_interval_sec_;
  double raw_log_fsync_interval_sec_;
  std::string raw_log_target_path_;
  std::string driver_boot_time_;

  // Serial
  int fd_;
  std::atomic<bool> running_;
  std::thread serial_thread_;
  uint64_t serial_open_count_{0};

  // Statistics
  std::atomic<uint64_t> msg_count_{0};
  std::atomic<uint64_t> good_fixes_{0};
  std::atomic<uint64_t> rejected_fixes_{0};
  std::atomic<uint64_t> checksum_errors_{0};
  std::atomic<uint64_t> serial_reconnects_{0};

  // Raw logging and health
  std::mutex raw_log_mutex_;
  std::ofstream raw_log_stream_;
  int raw_log_sync_fd_{-1};
  std::string current_raw_log_path_;
  std::atomic<bool> raw_logging_active_{false};
  std::atomic<uint64_t> raw_log_bytes_written_{0};
  std::string raw_log_last_error_;
  std::optional<std::chrono::steady_clock::time_point> raw_log_start_steady_;
  std::optional<std::chrono::system_clock::time_point> raw_log_start_wall_time_;
  std::string raw_log_start_iso_;
  std::optional<std::chrono::steady_clock::time_point> rawx_first_steady_;
  std::optional<std::chrono::steady_clock::time_point> rawx_last_steady_;
  std::optional<std::chrono::steady_clock::time_point> sfrbx_first_steady_;
  std::optional<std::chrono::steady_clock::time_point> sfrbx_last_steady_;
  std::optional<std::chrono::steady_clock::time_point> last_flush_steady_;
  std::optional<std::chrono::steady_clock::time_point> last_fsync_steady_;
  std::string rawx_first_iso_;
  std::string rawx_last_iso_;
  std::string sfrbx_first_iso_;
  std::string sfrbx_last_iso_;
  uint64_t rawx_count_{0};
  uint64_t sfrbx_count_{0};
  uint64_t nav_sat_count_{0};
  uint64_t hpposllh_count_{0};
  uint64_t logging_failures_{0};
  uint64_t flush_count_{0};
  uint64_t fsync_count_{0};
  double max_rawx_gap_sec_{0.0};
  bool rawx_startup_failed_{false};
  bool sfrbx_startup_warned_{false};
  bool rawx_gap_warned_{false};
  bool rawx_gap_invalid_{false};
  bool gnss_invalid_{false};
  std::vector<std::string> warning_reasons_;
  std::vector<std::string> invalid_reasons_;

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_health_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr raw_log_set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr raw_log_promote_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr raw_health_snapshot_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GpsDriver>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("gps_driver"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

