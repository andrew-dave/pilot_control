/**
 * @file gps_driver.cpp
 * @brief ROS2 driver for u-blox ZED-F9P GNSS receiver
 * 
 * Features:
 * - UBX protocol parsing (NAV-PVT)
 * - Quality gating with configurable thresholds
 * - Auto-reconnection on USB disconnect
 * - Receiver configuration on startup
 * - Publishes position, velocity, and diagnostics
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <algorithm>

// UBX Protocol Constants
constexpr uint8_t UBX_SYNC1 = 0xB5;
constexpr uint8_t UBX_SYNC2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_NAV_PVT = 0x07;
constexpr uint8_t UBX_CLASS_CFG = 0x06;

// Dynamic Model Values
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
  uint8_t valid;        // Validity flags (bit0=validDate, bit1=validTime, bit2=fullyResolved)
  uint32_t tAcc;        // Time accuracy estimate (ns)
  int32_t nano;         // Fraction of second (-1e9..1e9)
  uint8_t fixType;      // 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
  uint8_t flags;        // bit0=gnssFixOK, bit1=diffSoln, bit4-5=carrSoln
  uint8_t flags2;       // bit5=confirmedAvai, bit6=confirmedDate, bit7=confirmedTime
  uint8_t numSV;        // Number of satellites used in solution
  int32_t lon;          // Longitude (1e-7 degrees)
  int32_t lat;          // Latitude (1e-7 degrees)
  int32_t height;       // Height above ellipsoid (mm)
  int32_t hMSL;         // Height above mean sea level (mm)
  uint32_t hAcc;        // Horizontal accuracy estimate (mm)
  uint32_t vAcc;        // Vertical accuracy estimate (mm)
  int32_t velN;         // NED north velocity (mm/s)
  int32_t velE;         // NED east velocity (mm/s)
  int32_t velD;         // NED down velocity (mm/s)
  int32_t gSpeed;       // Ground speed (mm/s)
  int32_t headMot;      // Heading of motion (1e-5 degrees)
  uint32_t sAcc;        // Speed accuracy estimate (mm/s)
  uint32_t headAcc;     // Heading accuracy estimate (1e-5 degrees)
  uint16_t pDOP;        // Position DOP (0.01)
  uint8_t flags3;       // bit0=invalidLlh
  uint8_t reserved1[5];
  int32_t headVeh;      // Heading of vehicle (1e-5 degrees)
  int16_t magDec;       // Magnetic declination (1e-2 degrees)
  uint16_t magAcc;      // Magnetic declination accuracy (1e-2 degrees)
};
#pragma pack(pop)

static_assert(sizeof(UbxNavPvt) == 92, "UbxNavPvt size must be 92 bytes");

class GpsDriver : public rclcpp::Node {
public:
  GpsDriver() : Node("gps_driver"), fd_(-1), running_(false) {
    // Declare parameters
    this->declare_parameter<std::string>("device", "/dev/gps");
    this->declare_parameter<int>("baud_rate", 38400);
    this->declare_parameter<std::string>("frame_id", "gps_link");
    
    // Quality gating parameters (optimized for ZED-F9P)
    this->declare_parameter<double>("hacc_max_m", 5.0);
    this->declare_parameter<int>("min_sats", 8);
    this->declare_parameter<double>("pdop_max", 4.0);
    this->declare_parameter<bool>("require_3d_fix", true);
    this->declare_parameter<bool>("require_gnss_fix_ok", true);
    
    // Receiver configuration
    this->declare_parameter<bool>("configure_receiver", true);
    this->declare_parameter<int>("rate_hz", 5);
    this->declare_parameter<int>("dynamic_model", DYN_MODEL_AUTOMOTIVE);

    // Get parameters
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

    // Publishers
    fix_raw_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix_raw", 10);
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/gps/vel", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/gps/diag", 10);

    // Start serial thread
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
  }

  ~GpsDriver() override {
    running_ = false;
    if (serial_thread_.joinable()) serial_thread_.join();
    if (fd_ >= 0) close(fd_);
  }

private:
  bool openSerial() {
    // Try primary device, fall back to /dev/ttyACM0
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
      close(fd_); fd_ = -1;
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
    tty.c_cc[VTIME] = 1;  // 100ms read timeout
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(fd_); fd_ = -1;
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud", 
                device_.c_str(), baud_rate_);
    
    if (configure_receiver_) {
      configureReceiver();
    }
    
    return true;
  }

  void configureReceiver() {
    RCLCPP_INFO(this->get_logger(), "Configuring ZED-F9P receiver...");
    
    // Small delay to ensure port is ready
    usleep(100000);
    
    // 1. Set measurement rate
    uint16_t rate_ms = 1000 / rate_hz_;
    uint8_t set_rate[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x08,  // CFG-RATE
      0x06, 0x00,  // Length = 6
      static_cast<uint8_t>(rate_ms & 0xFF), 
      static_cast<uint8_t>((rate_ms >> 8) & 0xFF),
      0x01, 0x00,  // navRate = 1
      0x01, 0x00,  // timeRef = GPS
      0x00, 0x00   // Checksum
    };
    sendUbxMessage(set_rate, sizeof(set_rate));
    
    // 2. Set dynamic model using CFG-VALSET (ZED-F9P configuration)
    // CFG-NAVSPG-DYNMODEL key = 0x20110021
    uint8_t set_dyn[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x8A,  // CFG-VALSET
      0x09, 0x00,  // Length = 9
      0x00,        // Version 0
      0x01,        // Layer: RAM only
      0x00, 0x00,  // Reserved
      0x21, 0x00, 0x11, 0x20,  // Key: CFG-NAVSPG-DYNMODEL (little-endian)
      static_cast<uint8_t>(dynamic_model_),
      0x00, 0x00   // Checksum
    };
    sendUbxMessage(set_dyn, sizeof(set_dyn));
    
    // 3. Enable UBX-NAV-PVT output on USB (rate = 1)
    uint8_t enable_pvt[] = {
      UBX_SYNC1, UBX_SYNC2,
      0x06, 0x01,  // CFG-MSG
      0x03, 0x00,  // Length = 3
      0x01, 0x07,  // Class=NAV, ID=PVT
      0x01,        // Rate = 1 (every navigation solution)
      0x00, 0x00   // Checksum
    };
    sendUbxMessage(enable_pvt, sizeof(enable_pvt));
    
    // 4. Disable common NMEA messages to reduce serial traffic
    const uint8_t nmea_msgs[][2] = {
      {0xF0, 0x00},  // GGA
      {0xF0, 0x01},  // GLL
      {0xF0, 0x02},  // GSA
      {0xF0, 0x03},  // GSV
      {0xF0, 0x04},  // RMC
      {0xF0, 0x05},  // VTG
    };
    
    for (const auto& msg : nmea_msgs) {
      uint8_t disable_nmea[] = {
        UBX_SYNC1, UBX_SYNC2,
        0x06, 0x01,  // CFG-MSG
        0x03, 0x00,  // Length = 3
        msg[0], msg[1],
        0x00,        // Rate = 0 (disabled)
        0x00, 0x00   // Checksum
      };
      sendUbxMessage(disable_nmea, sizeof(disable_nmea));
    }
    
    usleep(200000);  // Wait 200ms for config to apply
    RCLCPP_INFO(this->get_logger(), 
        "Receiver configured: rate=%dHz, dynamic_model=%d, NMEA disabled, UBX-NAV-PVT enabled",
        rate_hz_, dynamic_model_);
  }

  void sendUbxMessage(uint8_t* msg, size_t len) {
    // Calculate checksum (exclude sync bytes)
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < len - 2; ++i) {
      ck_a += msg[i];
      ck_b += ck_a;
    }
    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;
    
    ssize_t written = write(fd_, msg, len);
    if (written != static_cast<ssize_t>(len)) {
      RCLCPP_WARN(this->get_logger(), "UBX write incomplete: %zd/%zu bytes", written, len);
    }
    usleep(50000);  // 50ms between commands
  }

  bool verifyChecksum(const std::vector<uint8_t>& frame) {
    if (frame.size() < 8) return false;
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < frame.size() - 2; ++i) {
      ck_a += frame[i];
      ck_b += ck_a;
    }
    return (frame[frame.size() - 2] == ck_a && frame[frame.size() - 1] == ck_b);
  }

  void serialLoop() {
    std::vector<uint8_t> buffer;
    buffer.reserve(4096);
    
    while (running_) {
      // Reconnect if needed
      if (fd_ < 0) {
        if (!openSerial()) {
          std::this_thread::sleep_for(std::chrono::seconds(2));
          continue;
        }
      }

      // Read available data
      uint8_t tmp[512];
      ssize_t n = read(fd_, tmp, sizeof(tmp));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
          close(fd_); fd_ = -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      
      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // Append to buffer
      buffer.insert(buffer.end(), tmp, tmp + n);

      // Parse UBX frames
      while (buffer.size() >= 8) {
        // Find UBX sync bytes
        auto it = std::find(buffer.begin(), buffer.end(), UBX_SYNC1);
        if (it == buffer.end()) {
          buffer.clear();
          break;
        }
        if (it != buffer.begin()) {
          buffer.erase(buffer.begin(), it);
        }
        
        if (buffer.size() < 2 || buffer[1] != UBX_SYNC2) {
          // Not a valid UBX frame start, skip this byte
          buffer.erase(buffer.begin());
          continue;
        }

        if (buffer.size() < 6) break;  // Need full header
        
        uint16_t payload_len = buffer[4] | (buffer[5] << 8);
        
        // Sanity check payload length
        if (payload_len > 1000) {
          buffer.erase(buffer.begin());
          continue;
        }
        
        size_t frame_len = 6 + payload_len + 2;  // Header + payload + checksum
        
        if (buffer.size() < frame_len) break;  // Incomplete frame
        
        // Extract complete frame
        std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_len);
        buffer.erase(buffer.begin(), buffer.begin() + frame_len);

        // Verify checksum
        if (!verifyChecksum(frame)) {
          checksum_errors_++;
          RCLCPP_DEBUG(this->get_logger(), "UBX checksum failed");
          continue;
        }

        // Process NAV-PVT messages
        if (frame[2] == UBX_CLASS_NAV && frame[3] == UBX_NAV_PVT && 
            payload_len >= sizeof(UbxNavPvt)) {
          processNavPvt(reinterpret_cast<const UbxNavPvt*>(&frame[6]));
        }
      }
      
      // Prevent buffer overflow
      if (buffer.size() > 8192) {
        RCLCPP_WARN(this->get_logger(), "Buffer overflow, clearing");
        buffer.clear();
      }
    }
  }

  void processNavPvt(const UbxNavPvt* pvt) {
    auto now = this->get_clock()->now();
    msg_count_++;
    
    // Extract quality indicators
    double h_acc_m = pvt->hAcc * 1e-3;
    double v_acc_m = pvt->vAcc * 1e-3;
    double pdop = pvt->pDOP * 0.01;
    bool gnss_fix_ok = (pvt->flags & 0x01) != 0;
    bool diff_soln = (pvt->flags & 0x02) != 0;
    uint8_t carr_soln = (pvt->flags >> 6) & 0x03;
    
    // Build NavSatFix message
    auto fix_msg = sensor_msgs::msg::NavSatFix();
    fix_msg.header.stamp = now;
    fix_msg.header.frame_id = frame_id_;
    
    fix_msg.latitude = pvt->lat * 1e-7;
    fix_msg.longitude = pvt->lon * 1e-7;
    fix_msg.altitude = pvt->hMSL * 1e-3;  // Height above MSL
    
    // Covariance (diagonal)
    fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_msg.position_covariance[0] = h_acc_m * h_acc_m;  // East variance
    fix_msg.position_covariance[4] = h_acc_m * h_acc_m;  // North variance
    fix_msg.position_covariance[8] = v_acc_m * v_acc_m;  // Up variance
    
    // Fix status
    if (pvt->fixType < 2) {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (diff_soln || carr_soln > 0) {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    } else {
      fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                             sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    
    // Always publish raw data
    fix_raw_pub_->publish(fix_msg);
    
    // Quality gating for filtered topic
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
    
    // Publish velocity (NED -> ENU conversion)
    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header = fix_msg.header;
    vel_msg.twist.linear.x = pvt->velE * 1e-3;   // East velocity (m/s)
    vel_msg.twist.linear.y = pvt->velN * 1e-3;   // North velocity (m/s)
    vel_msg.twist.linear.z = -pvt->velD * 1e-3;  // Up velocity (m/s)
    vel_pub_->publish(vel_msg);
    
    // Publish diagnostics (throttled to 1Hz)
    publishDiagnostics(now, pvt, h_acc_m, v_acc_m, pdop, gnss_fix_ok, 
                       diff_soln, carr_soln, passes_gate);
  }

  void publishDiagnostics(const rclcpp::Time& now, const UbxNavPvt* pvt,
                          double h_acc_m, double v_acc_m, double pdop,
                          bool gnss_fix_ok, bool diff_soln, uint8_t carr_soln,
                          bool quality_ok) {
    // Throttle to 1Hz using steady_clock (avoids time source mismatch issues)
    static auto last_diag_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_diag_time).count() < 1000) return;
    last_diag_time = current_time;
    
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
    
    auto add_kv = [&](const std::string& key, const std::string& val) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key; kv.value = val;
      ds.values.push_back(kv);
    };
    
    // Fix info
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
    
    // Accuracy
    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f m", h_acc_m);
    add_kv("h_accuracy", buf);
    snprintf(buf, sizeof(buf), "%.2f m", v_acc_m);
    add_kv("v_accuracy", buf);
    snprintf(buf, sizeof(buf), "%.2f", pdop);
    add_kv("pdop", buf);
    
    // Position
    snprintf(buf, sizeof(buf), "%.7f", pvt->lat * 1e-7);
    add_kv("latitude", buf);
    snprintf(buf, sizeof(buf), "%.7f", pvt->lon * 1e-7);
    add_kv("longitude", buf);
    snprintf(buf, sizeof(buf), "%.2f m", pvt->hMSL * 1e-3);
    add_kv("altitude_msl", buf);
    
    // Velocity
    snprintf(buf, sizeof(buf), "%.2f m/s", pvt->gSpeed * 1e-3);
    add_kv("ground_speed", buf);
    snprintf(buf, sizeof(buf), "%.1f deg", pvt->headMot * 1e-5);
    add_kv("heading", buf);
    
    // Statistics
    add_kv("messages_received", std::to_string(msg_count_.load()));
    add_kv("good_fixes", std::to_string(good_fixes_.load()));
    add_kv("rejected_fixes", std::to_string(rejected_fixes_.load()));
    add_kv("checksum_errors", std::to_string(checksum_errors_.load()));
    
    double accept_rate = msg_count_ > 0 ? 
        100.0 * good_fixes_ / msg_count_ : 0.0;
    snprintf(buf, sizeof(buf), "%.1f%%", accept_rate);
    add_kv("acceptance_rate", buf);
    
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

  // Serial
  int fd_;
  std::atomic<bool> running_;
  std::thread serial_thread_;

  // Statistics
  std::atomic<uint64_t> msg_count_{0};
  std::atomic<uint64_t> good_fixes_{0};
  std::atomic<uint64_t> rejected_fixes_{0};
  std::atomic<uint64_t> checksum_errors_{0};

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
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

