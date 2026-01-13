#include <seekcamera/seekcamera.h>
#include <seekcamera/seekcamera_manager.h>
#include <seekcamera/seekcamera_frame.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>

#include <opencv2/opencv.hpp>

// JPEG XL includes (conditional)
#if HAVE_JPEGXL
#include <jxl/encode.h>
#include <jxl/thread_parallel_runner.h>
#include <jxl/color_encoding.h>
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

// ==== SDK compat shims (handle older enum names) ====
#ifndef SEEKCAMERA_PIPELINE_MODE_IMAGE_LITE
  #ifdef SEEKCAMERA_IMAGE_LITE
    #define SEEKCAMERA_PIPELINE_MODE_IMAGE_LITE SEEKCAMERA_IMAGE_LITE
  #endif
#endif
#ifndef SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY
  #ifdef SEEKCAMERA_IMAGE_LEGACY
    #define SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY SEEKCAMERA_IMAGE_LEGACY
  #endif
#endif
#ifndef SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION
  #ifdef SEEKCAMERA_IMAGE_SEEKVISION
    #define SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION SEEKCAMERA_IMAGE_SEEKVISION
  #endif
#endif

// ========================= Config =========================
struct Config {
  std::string odom_topic      = "/Odometry_tilt_corrected_diff";  // Use tilt-corrected odometry from diff_drive_controller
  std::string log_directory   = std::string(getenv("HOME")?getenv("HOME"):".") + "/unified_scans";
  bool use_seekvision_mode    = true;   // SeekVision pipeline for colorized stream
  bool save_color_png         = true;   // write *_color.png alongside float32 bin
  int  csv_flush_every_rows   = 50;     // batch CSV flush, no per-row flush
  size_t frame_ring_size      = 24;     // ~2–3 s @ 9 Hz camera
  double log_frequency_hz     = 5.0;    // logging frequency in Hz
  
  // Image compression params
  std::string image_compression = "jxl";  // "png", "jpeg", "webp", "jxl"
  int jxl_effort              = 3;        // 1-9, 3=fast, 7=balanced, 9=slowest
  double jxl_distance         = 1.0;      // 0.0=lossless, 1.0=visually lossless, higher=more lossy
  bool jxl_fast_mode          = true;     // Enable speed optimizations
  
  // Video streaming params
  std::string left_device     = "/dev/v4l/by-id/See3CAM_Left-video-index0";
  std::string right_device    = "/dev/v4l/by-id/See3CAM_Right-video-index0";
  std::string stream_host     = "172.16.10.121";
  int stream_port            = 5600;
  int stream_bitrate_kbps    = 800;
  int rtp_mtu               = 1200;
  bool use_mjpeg_pipeline    = true;
  int cap_w                 = 1920;
  int cap_h                 = 1080;
  int cap_fps               = 30;
  std::string raw_format     = "UYVY";
  int raw_w                 = 1280;
  int raw_h                 = 720;
  int raw_fps               = 60;
};

// ==================== Frame containers ====================
struct ThermFrame {
  uint64_t ts_ns = 0;  // SDK UTC timestamp (ns)
  int w = 0, h = 0;
  std::vector<float> thermo;   // °C, size w*h
  cv::Mat color_bgr;           // colorized display (optional)
};

struct CameraFrame {
  uint64_t ts_ns = 0;  // timestamp (ns)
  cv::Mat image;       // BGR image
  std::string camera_label;
};

// GPS data structure
struct GpsFrame {
  uint64_t ts_ns = 0;           // ROS timestamp (ns)
  double latitude = 0.0;        // degrees
  double longitude = 0.0;       // degrees
  double altitude = 0.0;        // meters (MSL)
  double h_acc = 0.0;           // horizontal accuracy (m)
  double v_acc = 0.0;           // vertical accuracy (m)
  int8_t status = -1;           // -1=no data, 0=no fix, 1=fix, 2=SBAS/DGPS
  bool valid = false;           // true if fix is valid
};

class FrameRing {
public:
  explicit FrameRing(size_t cap) : cap_(cap) {}
  
  void push(ThermFrame&& f) {
    std::lock_guard<std::mutex> lk(m_);
    thermo_q_.emplace_back(std::move(f));
    while (thermo_q_.size() > cap_) thermo_q_.pop_front();
  }
  
  void push(CameraFrame&& f) {
    std::lock_guard<std::mutex> lk(m_);
    if (f.camera_label == "left") {
      left_q_.emplace_back(std::move(f));
      while (left_q_.size() > cap_) left_q_.pop_front();
    } else if (f.camera_label == "right") {
      right_q_.emplace_back(std::move(f));
      while (right_q_.size() > cap_) right_q_.pop_front();
    }
  }
  
  bool nearest_thermal(uint64_t target_ns, ThermFrame& out, uint64_t& dt_abs_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (thermo_q_.empty()) return false;
    size_t best = 0; uint64_t bestdt = UINT64_MAX;
    for (size_t i=0;i<thermo_q_.size();++i) {
      uint64_t a=thermo_q_[i].ts_ns,b=target_ns; uint64_t d=(a>b)?(a-b):(b-a);
      if (d<bestdt){ bestdt=d; best=i; }
    }
    out = thermo_q_[best]; dt_abs_ns = bestdt; return true;
  }
  
  bool nearest_camera(uint64_t target_ns, const std::string& camera, CameraFrame& out, uint64_t& dt_abs_ns) {
    std::lock_guard<std::mutex> lk(m_);
    std::deque<CameraFrame>* q = (camera == "left") ? &left_q_ : &right_q_;
    if (q->empty()) return false;
    size_t best = 0; uint64_t bestdt = UINT64_MAX;
    for (size_t i=0;i<q->size();++i) {
      uint64_t a=(*q)[i].ts_ns,b=target_ns; uint64_t d=(a>b)?(a-b):(b-a);
      if (d<bestdt){ bestdt=d; best=i; }
    }
    out = (*q)[best]; dt_abs_ns = bestdt; return true;
  }
  
  void push(GpsFrame&& f) {
    std::lock_guard<std::mutex> lk(m_);
    gps_q_.emplace_back(std::move(f));
    while (gps_q_.size() > cap_) gps_q_.pop_front();
  }
  
  bool nearest_gps(uint64_t target_ns, GpsFrame& out, uint64_t& dt_abs_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (gps_q_.empty()) return false;
    size_t best = 0; uint64_t bestdt = UINT64_MAX;
    for (size_t i = 0; i < gps_q_.size(); ++i) {
      uint64_t a = gps_q_[i].ts_ns, b = target_ns;
      uint64_t d = (a > b) ? (a - b) : (b - a);
      if (d < bestdt) { bestdt = d; best = i; }
    }
    out = gps_q_[best]; dt_abs_ns = bestdt; return true;
  }

private:
  std::mutex m_;
  std::deque<ThermFrame> thermo_q_;
  std::deque<CameraFrame> left_q_;
  std::deque<CameraFrame> right_q_;
  std::deque<GpsFrame> gps_q_;
  size_t cap_;
};

// ==================== JPEG XL Compression Helper (Speed Optimized) ====================
#if HAVE_JPEGXL
static bool compress_opencv_to_jxl(const cv::Mat& image, std::vector<uint8_t>& compressed_data, 
                                  int effort = 3, double /*distance*/ = 1.0) {
  // Speed optimizations:
  // - effort=3 (fast encoding)
  // - No BGR->RGB conversion (direct BGR encoding)
  // - Limited threads (max 4) to avoid context switching overhead
  // - Smaller initial buffer (32KB vs 64KB)
  // - Fast decoding speed setting
  // - No resampling
  if (image.empty() || image.channels() != 3) return false;
  
  // Create encoder
  JxlEncoder* encoder = JxlEncoderCreate(nullptr);
  if (!encoder) return false;
  
  // Set up parallel runner for better performance (limit to 4 threads for speed)
  int num_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
  void* runner = JxlThreadParallelRunnerCreate(nullptr, num_threads);
  JxlEncoderSetParallelRunner(encoder, JxlThreadParallelRunner, runner);
  
  // Configure basic info
  JxlBasicInfo basic_info;
  JxlEncoderInitBasicInfo(&basic_info);
  basic_info.xsize = image.cols;
  basic_info.ysize = image.rows;
  basic_info.bits_per_sample = 8;
  basic_info.num_color_channels = 3;
  basic_info.alpha_bits = 0;
  basic_info.alpha_exponent_bits = 0;
  basic_info.uses_original_profile = JXL_FALSE;
  
  JxlEncoderSetBasicInfo(encoder, &basic_info);
  
  // Set color encoding (sRGB)
  JxlColorEncoding color_encoding = {};
  JxlColorEncodingSetToSRGB(&color_encoding, JXL_FALSE);
  JxlEncoderSetColorEncoding(encoder, &color_encoding);
  
  // Configure frame settings
  JxlEncoderFrameSettings* frame_settings = JxlEncoderFrameSettingsCreate(encoder, nullptr);
  
  // Set compression parameters for speed (libjxl 0.12.0 compatible)
  JxlEncoderFrameSettingsSetOption(frame_settings, JXL_ENC_FRAME_SETTING_EFFORT, effort);
  
  // For libjxl 0.12.0, we'll use effort level to control quality/speed balance
  // Lower effort = faster encoding, higher effort = better compression
  // The distance parameter is handled through effort level in this version
  
  // Use BGR directly (avoid conversion) - JPEG XL can handle BGR
  JxlPixelFormat pixel_format = {3, JXL_TYPE_UINT8, JXL_NATIVE_ENDIAN, 0};
  
  // Add image frame directly (no BGR->RGB conversion needed)
  if (JxlEncoderAddImageFrame(frame_settings, &pixel_format, 
                             image.data, image.total() * image.elemSize()) != JXL_ENC_SUCCESS) {
    JxlThreadParallelRunnerDestroy(runner);
    JxlEncoderDestroy(encoder);
    return false;
  }
  
  JxlEncoderCloseInput(encoder);
  
  // Process output with smaller initial buffer for speed
  compressed_data.resize(32 * 1024); // Smaller initial buffer for faster allocation
  uint8_t* next_out = compressed_data.data();
  size_t avail_out = compressed_data.size();
  
  JxlEncoderStatus status = JXL_ENC_NEED_MORE_OUTPUT;
  while (status == JXL_ENC_NEED_MORE_OUTPUT) {
    status = JxlEncoderProcessOutput(encoder, &next_out, &avail_out);
    if (status == JXL_ENC_NEED_MORE_OUTPUT) {
      size_t offset = next_out - compressed_data.data();
      compressed_data.resize(compressed_data.size() * 2);
      next_out = compressed_data.data() + offset;
      avail_out = compressed_data.size() - offset;
    }
  }
  
  compressed_data.resize(next_out - compressed_data.data());
  
  // Cleanup
  JxlThreadParallelRunnerDestroy(runner);
  JxlEncoderDestroy(encoder);
  
  return status == JXL_ENC_SUCCESS;
}
#else
// Fallback: JPEG XL not available, return false
static bool compress_opencv_to_jxl(const cv::Mat& image, std::vector<uint8_t>& compressed_data, 
                                  int effort = 3, double distance = 1.0) {
  (void)image; (void)compressed_data; (void)effort; (void)distance; // Suppress unused warnings
  return false;
}
#endif

// ==================== Async writer ====================
struct CsvJob {
  uint64_t odom_ns{}, cam_ns{}, left_ns{}, right_ns{}, gps_ns{};
  double dt_thermal_ms{}, dt_left_ms{}, dt_right_ms{}, dt_gps_ms{};
  // pose / orientation
  double px{},py{},pz{}, qx{},qy{},qz{},qw{};
  // twist
  double vx{},vy{},vz{}, wx{},wy{},wz{};
  // GPS data
  GpsFrame gps_frame;
  // data
  ThermFrame thermal_frame;
  CameraFrame left_frame;
  CameraFrame right_frame;
  fs::path out_dir;                            // session_dir/frames
  std::shared_ptr<std::ofstream> csv;          // session_dir/unified_log.csv
  // JPEG XL compression parameters
  int jxl_effort = 7;
  double jxl_distance = 1.0;
  // Camera switch flag (marks rows captured during camera switch transition)
  bool camera_switching = false;
  std::string streaming_camera = "left";  // Which camera is currently streaming
};

class CsvWriter {
public:
  explicit CsvWriter(int flush_every_rows) : flush_every_rows_(flush_every_rows) {}
  ~CsvWriter(){ stop(); }

  void start(){ running_=true; th_=std::thread([this]{ run(); }); }
  void stop(){ running_=false; cv_.notify_all(); if (th_.joinable()) th_.join(); }
  void enqueue(CsvJob&& j){ { std::lock_guard<std::mutex> lk(m_); q_.emplace_back(std::move(j)); } cv_.notify_one(); }

private:
  static std::string save_float32_bin(const ThermFrame& f, const fs::path& dir, const std::string& stem) {
    fs::create_directories(dir);
    std::string filename = stem + "_thermo_f32.bin";
    fs::path p = dir / filename;
    std::ofstream ofs(p, std::ios::binary);
    int32_t w=f.w,h=f.h;
    ofs.write(reinterpret_cast<const char*>(&w),4);
    ofs.write(reinterpret_cast<const char*>(&h),4);
    ofs.write(reinterpret_cast<const char*>(f.thermo.data()), f.w*f.h*sizeof(float));
    return "frames/" + filename;  // Return relative path
  }
  
  static std::string save_color_png(const ThermFrame& f, const fs::path& dir, const std::string& stem) {
    if (f.color_bgr.empty()) return "";
    fs::create_directories(dir);
    std::string filename = stem + "_thermal_color.png";
    fs::path p = dir / filename;
    cv::imwrite(p.string(), f.color_bgr);
    return "frames/" + filename;  // Return relative path
  }
  
  static std::string save_camera_jxl(const CameraFrame& f, const fs::path& dir, const std::string& stem, 
                                    int effort = 7, double distance = 1.0) {
    if (f.image.empty()) return "";
    fs::create_directories(dir);
    
#if HAVE_JPEGXL
    std::string filename = stem + "_" + f.camera_label + ".jxl";
    fs::path p = dir / filename;
    
    // Compress using JPEG XL
    std::vector<uint8_t> compressed_data;
    if (compress_opencv_to_jxl(f.image, compressed_data, effort, distance)) {
      std::ofstream file(p, std::ios::binary);
      file.write(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
      return "frames/" + filename;  // Return relative path
    }
    return "";
#else
    // JPEG XL not available - skip camera image saving
    return "";
#endif
  }

  void run(){
    int rows_since_flush = 0;
    while (running_) {
      CsvJob job;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk,[&]{ return !q_.empty() || !running_; });
        if (!running_ && q_.empty()) break;
        job = std::move(q_.front()); q_.pop_front();
      }
      
      std::ostringstream s; s << "odom" << job.odom_ns << "_thermal" << job.cam_ns 
                             << "_left" << job.left_ns << "_right" << job.right_ns;
      const std::string stem = s.str();

      const std::string thermo_bin = save_float32_bin(job.thermal_frame, job.out_dir, stem);
      std::string thermal_color_png;
      if (!job.thermal_frame.color_bgr.empty()) thermal_color_png = save_color_png(job.thermal_frame, job.out_dir, stem);
      
      const std::string left_jxl = save_camera_jxl(job.left_frame, job.out_dir, stem, job.jxl_effort, job.jxl_distance);
      const std::string right_jxl = save_camera_jxl(job.right_frame, job.out_dir, stem, job.jxl_effort, job.jxl_distance);

      auto& csv = *job.csv;
      csv << job.odom_ns << "," << job.cam_ns << "," << job.left_ns << "," << job.right_ns << ","
          << job.gps_ns << ","
          << std::fixed << std::setprecision(3) << job.dt_thermal_ms << ","
          << std::fixed << std::setprecision(3) << job.dt_left_ms << ","
          << std::fixed << std::setprecision(3) << job.dt_right_ms << ","
          << std::fixed << std::setprecision(3) << job.dt_gps_ms << ","
          << job.px << "," << job.py << "," << job.pz << ","
          << job.qx << "," << job.qy << "," << job.qz << "," << job.qw << ","
          << job.vx << "," << job.vy << "," << job.vz << ","
          << job.wx << "," << job.wy << "," << job.wz << ","
          << std::fixed << std::setprecision(9) << job.gps_frame.latitude << ","
          << std::fixed << std::setprecision(9) << job.gps_frame.longitude << ","
          << std::fixed << std::setprecision(3) << job.gps_frame.altitude << ","
          << std::fixed << std::setprecision(3) << job.gps_frame.h_acc << ","
          << std::fixed << std::setprecision(3) << job.gps_frame.v_acc << ","
          << (int)job.gps_frame.status << ","
          << thermo_bin << "," << thermal_color_png << ","
          << left_jxl << "," << right_jxl << ","
          << (job.camera_switching ? "1" : "0") << "," << job.streaming_camera << "\n";

      if (++rows_since_flush >= flush_every_rows_) { csv.flush(); rows_since_flush = 0; }
    }
  }

  std::thread th_;
  std::mutex m_; std::condition_variable cv_; std::deque<CsvJob> q_;
  std::atomic<bool> running_{false};
  int flush_every_rows_;
};

// ==================== Unified Node ====================
class UnifiedDataCollector : public rclcpp::Node {
public:
  UnifiedDataCollector()
      : rclcpp::Node("unified_data_collector"),
        recording_active_(false),
        shutting_down_(false),
        last_log_time_(std::chrono::steady_clock::now()),
        ring_(cfg_.frame_ring_size),
        writer_(cfg_.csv_flush_every_rows) {
    
    // Parameters
    declare_parameters();
    get_parameters();

    // Service for recording control
    record_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/video_record_set",
        std::bind(&UnifiedDataCollector::onSetRecording, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service ready: /video_record_set (std_srvs/SetBool)");
    
    // Camera stream selection subscriber (for switching between left/right cameras)
    camera_select_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/stream_camera_select", 10,
        std::bind(&UnifiedDataCollector::onCameraSelect, this, std::placeholders::_1));
    
    // Camera status publisher (publishes current streaming camera)
    camera_status_pub_ = this->create_publisher<std_msgs::msg::String>("/stream_camera_status", 10);
    RCLCPP_INFO(this->get_logger(), "Camera selection topic: /stream_camera_select (\"left\" or \"right\")");
    
    // Stream target IP subscriber (for dynamic stream destination)
    stream_target_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/stream_target_ip", 10,
        std::bind(&UnifiedDataCollector::onStreamTargetReceived, this, std::placeholders::_1));
    
    // Stream status publisher (publishes full stream configuration)
    stream_status_pub_ = this->create_publisher<std_msgs::msg::String>("/stream_status", 10);
    RCLCPP_INFO(this->get_logger(), "Stream target topic: /stream_target_ip (publish laptop IP to configure)");
    
    // Publish initial stream status
    publishStreamStatus();

    // ROS wiring
    // Use default QoS (reliable) for odometry - matches odom_tilt_corrector.py publisher QoS
    // SensorDataQoS (best-effort) was causing QoS mismatch with the reliable publisher
    auto odom_qos = rclcpp::QoS(100).reliable();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(cfg_.odom_topic, odom_qos,
                 std::bind(&UnifiedDataCollector::onOdom, this, std::placeholders::_1));
    
    // GPS uses SensorDataQoS since gps_driver publishes with sensor data QoS
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", sensor_qos,
                 std::bind(&UnifiedDataCollector::onGps, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "GPS subscription: /gps/fix (quality-gated)");

    // Initialize components
    initGStreamer();
    createCameraManager();
    writer_.start();

    RCLCPP_INFO(get_logger(), "UnifiedDataCollector ready. Odom: %s", cfg_.odom_topic.c_str());
    RCLCPP_INFO(get_logger(), "Call /video_record_set service to start/stop recording");
  }

  ~UnifiedDataCollector() override {
    shutting_down_.store(true);
    writer_.stop();
    if (csv_stream_ && csv_stream_->is_open()) { csv_stream_->flush(); csv_stream_->close(); }
    destroyCameraManager();
    stopGStreamer();
  }

private:
  void declare_parameters() {
    // Thermal/odometry params
    this->declare_parameter<std::string>("fastlio_odom_topic", cfg_.odom_topic);
    this->declare_parameter<std::string>("log_directory", cfg_.log_directory);
    this->declare_parameter<std::string>("visual_data_directory", "");  // Session-specific Visual_data folder
    this->declare_parameter<bool>("use_seekvision_mode", cfg_.use_seekvision_mode);
    this->declare_parameter<bool>("save_color_png", cfg_.save_color_png);
    this->declare_parameter<int>("csv_flush_every_rows", cfg_.csv_flush_every_rows);
    this->declare_parameter<double>("log_frequency_hz", cfg_.log_frequency_hz);
    
    // Image compression params
    this->declare_parameter<std::string>("image_compression", cfg_.image_compression);
    this->declare_parameter<int>("jxl_effort", cfg_.jxl_effort);
    this->declare_parameter<double>("jxl_distance", cfg_.jxl_distance);
    this->declare_parameter<bool>("jxl_fast_mode", cfg_.jxl_fast_mode);
    
    // Video streaming params
    this->declare_parameter<std::string>("left_device", cfg_.left_device);
    this->declare_parameter<std::string>("right_device", cfg_.right_device);
    this->declare_parameter<std::string>("stream_host", cfg_.stream_host);
    this->declare_parameter<int>("stream_port", cfg_.stream_port);
    this->declare_parameter<int>("stream_bitrate_kbps", cfg_.stream_bitrate_kbps);
    this->declare_parameter<int>("rtp_mtu", cfg_.rtp_mtu);
    this->declare_parameter<bool>("use_mjpeg_pipeline", cfg_.use_mjpeg_pipeline);
    this->declare_parameter<int>("cap_w", cfg_.cap_w);
    this->declare_parameter<int>("cap_h", cfg_.cap_h);
    this->declare_parameter<int>("cap_fps", cfg_.cap_fps);
    this->declare_parameter<std::string>("raw_format", cfg_.raw_format);
    this->declare_parameter<int>("raw_w", cfg_.raw_w);
    this->declare_parameter<int>("raw_h", cfg_.raw_h);
    this->declare_parameter<int>("raw_fps", cfg_.raw_fps);
  }
  
  void get_parameters() {
    this->get_parameter("fastlio_odom_topic", cfg_.odom_topic);
    this->get_parameter("log_directory", cfg_.log_directory);
    
    // Check for session-specific visual data directory
    std::string visual_data_dir;
    this->get_parameter("visual_data_directory", visual_data_dir);
    if (!visual_data_dir.empty()) {
      cfg_.log_directory = visual_data_dir;  // Override with session-specific path
    }
    
    this->get_parameter("use_seekvision_mode", cfg_.use_seekvision_mode);
    this->get_parameter("save_color_png", cfg_.save_color_png);
    this->get_parameter("csv_flush_every_rows", cfg_.csv_flush_every_rows);
    this->get_parameter("log_frequency_hz", cfg_.log_frequency_hz);
    
    this->get_parameter("image_compression", cfg_.image_compression);
    this->get_parameter("jxl_effort", cfg_.jxl_effort);
    this->get_parameter("jxl_distance", cfg_.jxl_distance);
    this->get_parameter("jxl_fast_mode", cfg_.jxl_fast_mode);
    
    this->get_parameter("left_device", cfg_.left_device);
    this->get_parameter("right_device", cfg_.right_device);
    this->get_parameter("stream_host", cfg_.stream_host);
    this->get_parameter("stream_port", cfg_.stream_port);
    this->get_parameter("stream_bitrate_kbps", cfg_.stream_bitrate_kbps);
    this->get_parameter("rtp_mtu", cfg_.rtp_mtu);
    this->get_parameter("use_mjpeg_pipeline", cfg_.use_mjpeg_pipeline);
    this->get_parameter("cap_w", cfg_.cap_w);
    this->get_parameter("cap_h", cfg_.cap_h);
    this->get_parameter("cap_fps", cfg_.cap_fps);
    this->get_parameter("raw_format", cfg_.raw_format);
    this->get_parameter("raw_w", cfg_.raw_w);
    this->get_parameter("raw_h", cfg_.raw_h);
    this->get_parameter("raw_fps", cfg_.raw_fps);
  }

  // ---------- GPS callback ----------
  void onGps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    GpsFrame f;
    f.ts_ns = (uint64_t)msg->header.stamp.sec * 1000000000ULL
            + (uint64_t)msg->header.stamp.nanosec;
    f.latitude = msg->latitude;
    f.longitude = msg->longitude;
    f.altitude = msg->altitude;
    
    // Extract accuracy from covariance (diagonal elements are variances)
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      f.h_acc = std::sqrt(msg->position_covariance[0]);  // sqrt of lat/lon variance
      f.v_acc = std::sqrt(msg->position_covariance[8]);  // sqrt of alt variance
    }
    
    f.status = msg->status.status;
    f.valid = (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    
    ring_.push(std::move(f));
  }

  // ---------- Odom -> enqueue one row ----------
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!recording_active_) return;
    
    // Throttle to configured frequency
    auto now = std::chrono::steady_clock::now();
    auto min_interval_ms = static_cast<int>(1000.0 / cfg_.log_frequency_hz);
    if (now - last_log_time_ < std::chrono::milliseconds(min_interval_ms)) {
      return; // Skip this odometry message
    }
    last_log_time_ = now;
    
    const uint64_t odom_ns = (uint64_t)msg->header.stamp.sec*1000000000ULL
                           + (uint64_t)msg->header.stamp.nanosec;

    ThermFrame thermal_f; uint64_t dt_thermal=0;
    CameraFrame left_f; uint64_t dt_left=0;
    CameraFrame right_f; uint64_t dt_right=0;
    GpsFrame gps_f; uint64_t dt_gps=0;
    
    if (!ring_.nearest_thermal(odom_ns, thermal_f, dt_thermal)) return;
    if (!ring_.nearest_camera(odom_ns, "left", left_f, dt_left)) return;
    if (!ring_.nearest_camera(odom_ns, "right", right_f, dt_right)) return;
    
    // GPS is optional - don't fail if no GPS data available
    bool has_gps = ring_.nearest_gps(odom_ns, gps_f, dt_gps);

    CsvJob job;
    job.odom_ns = odom_ns;
    job.cam_ns  = thermal_f.ts_ns;
    job.left_ns = left_f.ts_ns;
    job.right_ns = right_f.ts_ns;
    job.dt_thermal_ms = (double)dt_thermal / 1e6;
    job.dt_left_ms = (double)dt_left / 1e6;
    job.dt_right_ms = (double)dt_right / 1e6;
    
    // GPS data (optional)
    if (has_gps) {
      job.gps_ns = gps_f.ts_ns;
      job.dt_gps_ms = (double)dt_gps / 1e6;
      job.gps_frame = std::move(gps_f);
    } else {
      job.gps_ns = 0;
      job.dt_gps_ms = -1.0;  // Indicates no GPS data
      job.gps_frame = GpsFrame{};  // Empty/invalid frame
    }
    
    job.thermal_frame = std::move(thermal_f);
    job.left_frame = std::move(left_f);
    job.right_frame = std::move(right_f);
    job.out_dir = session_dir_ / "frames";
    job.csv = csv_stream_;
    job.jxl_effort = cfg_.jxl_effort;
    job.jxl_distance = cfg_.jxl_distance;

    // Pose
    job.px = msg->pose.pose.position.x;
    job.py = msg->pose.pose.position.y;
    job.pz = msg->pose.pose.position.z;
    job.qx = msg->pose.pose.orientation.x;
    job.qy = msg->pose.pose.orientation.y;
    job.qz = msg->pose.pose.orientation.z;
    job.qw = msg->pose.pose.orientation.w;

    // Twist
    job.vx = msg->twist.twist.linear.x;
    job.vy = msg->twist.twist.linear.y;
    job.vz = msg->twist.twist.linear.z;
    job.wx = msg->twist.twist.angular.x;
    job.wy = msg->twist.twist.angular.y;
    job.wz = msg->twist.twist.angular.z;

    // Set camera switch flag if within grace period
    job.camera_switching = camera_switching_.load();
    if (!job.camera_switching) {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - camera_switch_start_).count();
      if (elapsed < kCameraSwitchGracePeriodMs) {
        job.camera_switching = true;
      }
    }
    job.streaming_camera = streaming_right_camera_.load() ? "right" : "left";

    writer_.enqueue(std::move(job));
  }

  // ---------- Camera selection ----------
  void onCameraSelect(const std_msgs::msg::String::SharedPtr msg) {
    std::string camera = msg->data;
    std::transform(camera.begin(), camera.end(), camera.begin(), ::tolower);
    
    bool want_right = (camera == "right" || camera == "r");
    bool currently_right = streaming_right_camera_.load();
    
    if (want_right == currently_right) {
      RCLCPP_INFO(this->get_logger(), "Already streaming %s camera", camera.c_str());
      publishCameraStatus();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Switching stream to %s camera...", want_right ? "RIGHT" : "LEFT");
    
    // Set switching flag and timestamp
    camera_switching_.store(true);
    camera_switch_start_ = std::chrono::steady_clock::now();
    
    // Fully stop the existing GStreamer pipeline and main loop
    stopStreamingLoop();
    
    // Update which camera to stream
    streaming_right_camera_.store(want_right);
    
    // Rebuild and restart pipeline
    try {
      buildStreamingPipeline();
      startStreamingLoop();
      RCLCPP_INFO(this->get_logger(), "Successfully switched to %s camera", want_right ? "RIGHT" : "LEFT");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to switch camera: %s", e.what());
    }
    
    // Clear switching flag (grace period still applies via timestamp)
    camera_switching_.store(false);
    
    publishCameraStatus();
  }
  
  void stopStreamingLoop() {
    // Stop the pipeline first
    if (pipeline_) {
      gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
    }
    
    // Quit the main loop
    if (loop_) {
      g_main_loop_quit(loop_);
    }
    
    // Wait for loop thread to finish
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
    
    // Clean up GLib resources
    if (bus_watch_id_) {
      // Already removed in the thread, just reset
      bus_watch_id_ = 0;
    }
    if (loop_) {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }
    if (context_) {
      g_main_context_unref(context_);
      context_ = nullptr;
    }
    if (bus_) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    
    // Reset pipeline
    pipeline_.reset();
    left_appsink_ = nullptr;
    right_appsink_ = nullptr;
  }
  
  void publishCameraStatus() {
    auto msg = std_msgs::msg::String();
    msg.data = streaming_right_camera_.load() ? "right" : "left";
    camera_status_pub_->publish(msg);
  }
  
  // ---------- Stream target configuration ----------
  void onStreamTargetReceived(const std_msgs::msg::String::SharedPtr msg) {
    std::string new_host = msg->data;
    
    // Trim whitespace
    new_host.erase(0, new_host.find_first_not_of(" \t\n\r"));
    new_host.erase(new_host.find_last_not_of(" \t\n\r") + 1);
    
    if (new_host.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty stream target IP, ignoring");
      return;
    }
    
    // Check if IP changed
    if (new_host == cfg_.stream_host) {
      RCLCPP_INFO(this->get_logger(), "Stream target unchanged: %s", new_host.c_str());
      publishStreamStatus();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Updating stream target: %s -> %s", 
                cfg_.stream_host.c_str(), new_host.c_str());
    
    // Update config
    std::string old_host = cfg_.stream_host;
    cfg_.stream_host = new_host;
    
    // Set switching flag and timestamp
    camera_switching_.store(true);
    camera_switch_start_ = std::chrono::steady_clock::now();
    
    // Rebuild pipeline with new target
    try {
      stopStreamingLoop();
      buildStreamingPipeline();
      startStreamingLoop();
      RCLCPP_INFO(this->get_logger(), "Successfully updated stream target to: %s:%d", 
                  cfg_.stream_host.c_str(), cfg_.stream_port);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to update stream target: %s", e.what());
      // Revert to old host on failure
      cfg_.stream_host = old_host;
    }
    
    camera_switching_.store(false);
    publishStreamStatus();
  }
  
  void publishStreamStatus() {
    auto msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << cfg_.stream_host << ":" << cfg_.stream_port << ":" 
        << (streaming_right_camera_.load() ? "right" : "left");
    msg.data = oss.str();
    stream_status_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Stream status: %s", msg.data.c_str());
  }

  // ---------- Recording control ----------
  void onSetRecording(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    if (req->data) {
      if (recording_active_) {
        resp->success = true;
        resp->message = "Already recording to: " + session_dir_.string();
        return;
      }
      
      // Create new session directory with timestamp
      session_dir_ = fs::path(cfg_.log_directory) / ("dataset_" + timestampStr());
      fs::create_directories(session_dir_ / "frames");
      
      // Create and open CSV file
      csv_stream_ = std::make_shared<std::ofstream>((session_dir_ / "unified_log.csv").string(),
                                                    std::ios::out | std::ios::trunc);
      if (!csv_stream_->is_open()) {
        resp->success = false;
        resp->message = "Failed to create CSV file";
        RCLCPP_ERROR(get_logger(), "Failed to create CSV file: %s", (session_dir_ / "unified_log.csv").c_str());
        return;
      }
      
      // Write CSV header
      *csv_stream_ << "odom_stamp_ns,thermal_stamp_ns,left_stamp_ns,right_stamp_ns,gps_stamp_ns,"
                      "dt_thermal_ms,dt_left_ms,dt_right_ms,dt_gps_ms,"
                      "px,py,pz,qx,qy,qz,qw,"
                      "vx,vy,vz,wx,wy,wz,"
                      "gps_lat,gps_lon,gps_alt,gps_h_acc,gps_v_acc,gps_status,"
                      "thermo_f32_bin,thermal_color_png,left_image_jxl,right_image_jxl,"
                      "camera_switching,streaming_camera\n";
      csv_stream_->flush();
      
      recording_active_ = true;
      resp->success = true;
      resp->message = "Recording started to: " + session_dir_.string();
      RCLCPP_INFO(get_logger(), "Recording started - Session: %s", session_dir_.c_str());
    } else {
      if (!recording_active_) {
        resp->success = true;
        resp->message = "Already stopped";
        return;
      }
      recording_active_ = false;
      
      // Close CSV file
      if (csv_stream_ && csv_stream_->is_open()) {
        csv_stream_->flush();
        csv_stream_->close();
      }
      
      resp->success = true;
      resp->message = "Recording stopped. Data saved to: " + session_dir_.string();
      RCLCPP_INFO(get_logger(), "Recording stopped - Data saved to: %s", session_dir_.c_str());
    }
  }

  // ---------- Thermal Camera plumbing ----------
  void createCameraManager() {
    if (seekcamera_manager_create(&mgr_, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS)
      throw std::runtime_error("Failed to create seekcamera manager");
    seekcamera_manager_register_event_callback(mgr_, &UnifiedDataCollector::onThermalEventStatic, this);
  }
  
  void destroyCameraManager() {
    if (thermal_cam_) seekcamera_capture_session_stop(thermal_cam_);
    if (mgr_) seekcamera_manager_destroy(&mgr_);
    thermal_cam_ = nullptr; mgr_ = nullptr;
  }

  static void onThermalEventStatic(seekcamera_t* cam, seekcamera_manager_event_t ev, seekcamera_error_t status, void* user){
    static_cast<UnifiedDataCollector*>(user)->onThermalEvent(cam, ev, status);
  }
  
  void onThermalEvent(seekcamera_t* cam, seekcamera_manager_event_t ev, seekcamera_error_t status){
    (void)status;
    if (ev == SEEKCAMERA_MANAGER_EVENT_CONNECT) {
      thermal_cam_ = cam;
      seekcamera_set_pipeline_mode(thermal_cam_, cfg_.use_seekvision_mode
                                        ? SEEKCAMERA_IMAGE_SEEKVISION
                                        : SEEKCAMERA_IMAGE_LEGACY);

      // Set color palette to Spectra (rainbow spectrum)
      seekcamera_set_color_palette(thermal_cam_, SEEKCAMERA_COLOR_PALETTE_SPECTRA);

      uint32_t fmts = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
      if (cfg_.save_color_png) fmts |= SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888;

      seekcamera_register_frame_available_callback(thermal_cam_, &UnifiedDataCollector::onThermalFrameStatic, this);
      auto err = seekcamera_capture_session_start(thermal_cam_, fmts);
      if (err == SEEKCAMERA_SUCCESS) RCLCPP_INFO(get_logger(), "[Thermal] Streaming.");
      else RCLCPP_ERROR(get_logger(), "[Thermal] capture start failed: %d", (int)err);
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_DISCONNECT) {
      RCLCPP_WARN(get_logger(), "[Thermal] Disconnected.");
      if (thermal_cam_) seekcamera_capture_session_stop(thermal_cam_);
      thermal_cam_ = nullptr;
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR) {
      seekcamera_store_calibration_data(cam, nullptr, nullptr, nullptr);
    }
  }

  static void onThermalFrameStatic(seekcamera_t* cam, seekcamera_frame_t* cam_frame, void* user){
    static_cast<UnifiedDataCollector*>(user)->onThermalFrame(cam, cam_frame);
  }
  
  void onThermalFrame(seekcamera_t* cam, seekcamera_frame_t* cam_frame){
    (void)cam;
    seekcamera_frame_lock(cam_frame);

    ThermFrame f;

    seekframe_t* therm=nullptr;
    if (seekcamera_frame_get_frame_by_format(cam_frame,
        SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &therm) == SEEKCAMERA_SUCCESS && therm) {
      const float* px = static_cast<const float*>(seekframe_get_data(therm));
      f.w = (int)seekframe_get_width(therm);
      f.h = (int)seekframe_get_height(therm);
      f.thermo.assign(px, px + f.w * f.h);

      if (auto hdr = static_cast<const seekcamera_frame_header_t*>(seekframe_get_header(therm)))
        f.ts_ns = hdr->timestamp_utc_ns;
    }

    if (cfg_.save_color_png) {
      seekframe_t* argb=nullptr;
      if (seekcamera_frame_get_frame_by_format(cam_frame,
          SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888, &argb) == SEEKCAMERA_SUCCESS && argb) {
        int w=(int)seekframe_get_width(argb), h=(int)seekframe_get_height(argb);
        const uint8_t* p = static_cast<const uint8_t*>(seekframe_get_data(argb));
        cv::Mat argb32(h,w,CV_8UC4,const_cast<uint8_t*>(p));
        std::vector<cv::Mat> ch; ch.reserve(4);
        cv::split(argb32, ch);
        std::vector<cv::Mat> bgra_channels = {ch[3],ch[2],ch[1],ch[0]}; // B,G,R,A
        cv::Mat bgra; cv::merge(bgra_channels, bgra);
        cv::cvtColor(bgra, f.color_bgr, cv::COLOR_BGRA2BGR);
      }
    }

    seekcamera_frame_unlock(cam_frame);
    if (!f.thermo.empty()) ring_.push(std::move(f));
  }

  // ---------- GStreamer plumbing ----------
  void initGStreamer() {
    static std::once_flag gst_once;
    std::call_once(gst_once, [] { gst_init(nullptr, nullptr); });
    
    buildStreamingPipeline();
    startStreamingLoop();
  }
  
  void stopGStreamer() {
    stopStreamingLoop();
  }

  void buildStreamingPipeline() {
    // Clean up existing appsink references
    left_appsink_ = nullptr;
    right_appsink_ = nullptr;
    
    std::ostringstream oss;
    
    bool stream_right = streaming_right_camera_.load();
    
    RCLCPP_INFO(this->get_logger(), "Camera devices: LEFT=%s RIGHT=%s",
                cfg_.left_device.c_str(), cfg_.right_device.c_str());
    RCLCPP_INFO(this->get_logger(), "Pipeline mode: %s, Resolution: %dx%d@%dfps, STREAMING: %s",
                cfg_.use_mjpeg_pipeline ? "MJPEG" : "RAW",
                cfg_.cap_w, cfg_.cap_h, cfg_.cap_fps,
                stream_right ? "RIGHT" : "LEFT");
    
    // LEFT camera
    if (cfg_.use_mjpeg_pipeline) {
      oss << "v4l2src name=v4l2src_left device=" << cfg_.left_device << " do-timestamp=true io-mode=mmap "
          << "! image/jpeg,width=" << cfg_.cap_w << ",height=" << cfg_.cap_h << ",framerate=" << cfg_.cap_fps << "/1 "
          << "! jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=I420 ";
    } else {
      oss << "v4l2src name=v4l2src_left device=" << cfg_.left_device << " do-timestamp=true io-mode=mmap "
          << "! video/x-raw,format=" << cfg_.raw_format << ",width=" << cfg_.raw_w << ",height=" << cfg_.raw_h << ",framerate=" << cfg_.raw_fps << "/1 "
          << "! videoconvert ! video/x-raw,format=I420 ";
    }
    oss << "! tee name=T_left ";

    // LEFT stream branch (only if streaming left)
    if (!stream_right) {
    oss << " T_left. ! queue leaky=downstream max-size-buffers=120 max-size-bytes=0 max-size-time=0 "
        << "! videorate ! video/x-raw,framerate=15/1 "
        << "! videoscale ! video/x-raw,width=640,height=480,format=I420 "
        << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << cfg_.stream_bitrate_kbps << " key-int-max=30 bframes=0 "
        << "! video/x-h264,stream-format=byte-stream,alignment=au "
        << "! rtph264pay pt=96 config-interval=1 mtu=" << cfg_.rtp_mtu << " "
        << "! udpsink host=" << cfg_.stream_host << " port=" << cfg_.stream_port << " sync=false ";
    }

    // LEFT frame capture branch (appsink)
    oss << " T_left. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
        << "! videoconvert ! video/x-raw,format=BGR "
        << "! appsink name=left_appsink emit-signals=true sync=false max-buffers=1 drop=true ";

    // RIGHT camera
    if (cfg_.use_mjpeg_pipeline) {
      oss << "\n"
          << "v4l2src name=v4l2src_right device=" << cfg_.right_device << " do-timestamp=true io-mode=mmap "
          << "! image/jpeg,width=" << cfg_.cap_w << ",height=" << cfg_.cap_h << ",framerate=" << cfg_.cap_fps << "/1 "
          << "! jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=I420 ";
    } else {
      oss << "\n"
          << "v4l2src name=v4l2src_right device=" << cfg_.right_device << " do-timestamp=true io-mode=mmap "
          << "! video/x-raw,format=" << cfg_.raw_format << ",width=" << cfg_.raw_w << ",height=" << cfg_.raw_h << ",framerate=" << cfg_.raw_fps << "/1 "
          << "! videoconvert ! video/x-raw,format=I420 ";
    }
    oss << "! tee name=T_right ";
    
    // RIGHT stream branch (only if streaming right)
    // Note: Right camera is mounted upside down, so we rotate 180 degrees for streaming
    if (stream_right) {
      oss << " T_right. ! queue leaky=downstream max-size-buffers=120 max-size-bytes=0 max-size-time=0 "
          << "! videorate ! video/x-raw,framerate=15/1 "
          << "! videoscale ! video/x-raw,width=640,height=480,format=I420 "
          << "! videoflip method=rotate-180 "  // Rotate 180° for upside-down camera
          << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << cfg_.stream_bitrate_kbps << " key-int-max=30 bframes=0 "
          << "! video/x-h264,stream-format=byte-stream,alignment=au "
          << "! rtph264pay pt=96 config-interval=1 mtu=" << cfg_.rtp_mtu << " "
          << "! udpsink host=" << cfg_.stream_host << " port=" << cfg_.stream_port << " sync=false ";
    }
    
    // RIGHT frame capture branch (appsink)
    oss << " T_right. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
        << "! videoconvert ! video/x-raw,format=BGR "
        << "! appsink name=right_appsink emit-signals=true sync=false max-buffers=1 drop=true ";

    pipeline_str_ = oss.str();
    RCLCPP_INFO(this->get_logger(), "GStreamer streaming config: host=%s port=%d bitrate=%dkbps mtu=%d",
                cfg_.stream_host.c_str(), cfg_.stream_port, cfg_.stream_bitrate_kbps, cfg_.rtp_mtu);
    RCLCPP_INFO(this->get_logger(), "Launching GStreamer pipeline:\n%s", pipeline_str_.c_str());

    GError* err = nullptr;
    GstElement* pipe = gst_parse_launch(pipeline_str_.c_str(), &err);
    if (!pipe || err) {
      if (err) {
        RCLCPP_FATAL(this->get_logger(), "gst_parse_launch error: %s", err->message);
        g_error_free(err);
      } else {
        RCLCPP_FATAL(this->get_logger(), "Failed to create pipeline");
      }
      if (pipe) gst_object_unref(pipe);
      throw std::runtime_error("Failed to create GStreamer pipeline");
    }
    pipeline_.reset(pipe);

    // Setup appsinks
    left_appsink_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "left_appsink"));
    right_appsink_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "right_appsink"));
    
    if (!left_appsink_ || !right_appsink_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to retrieve appsinks");
      throw std::runtime_error("Missing appsinks");
    }

    g_signal_connect(left_appsink_, "new-sample", G_CALLBACK(onNewLeftFrame), this);
    g_signal_connect(right_appsink_, "new-sample", G_CALLBACK(onNewRightFrame), this);
  }

  void startStreamingLoop() {
    bus_ = gst_element_get_bus(pipeline_.get());
    if (!bus_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get pipeline bus");
      throw std::runtime_error("No bus");
    }

    context_ = g_main_context_new();
    loop_ = g_main_loop_new(context_, FALSE);

    loop_thread_ = std::thread([this]() {
      g_main_context_push_thread_default(context_);
      bus_watch_id_ = gst_bus_add_watch(bus_, &UnifiedDataCollector::bus_func, this);
      auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(this->get_logger(), "Failed to set pipeline to PLAYING - check camera devices and network");
      } else if (ret == GST_STATE_CHANGE_ASYNC) {
        RCLCPP_INFO(this->get_logger(), "Pipeline state change pending (async)");
      } else {
        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started successfully");
      }
      g_main_loop_run(loop_);
      if (bus_watch_id_) {
        g_source_remove(bus_watch_id_);
        bus_watch_id_ = 0;
      }
      g_main_context_pop_thread_default(context_);
    });
  }

  static GstFlowReturn onNewLeftFrame(GstElement* appsink, gpointer user_data) {
    return static_cast<UnifiedDataCollector*>(user_data)->onNewFrame(appsink, "left");
  }
  
  static GstFlowReturn onNewRightFrame(GstElement* appsink, gpointer user_data) {
    return static_cast<UnifiedDataCollector*>(user_data)->onNewFrame(appsink, "right");
  }
  
  GstFlowReturn onNewFrame(GstElement* appsink, const std::string& camera) {
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* structure = gst_caps_get_structure(caps, 0);
    
    gint width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    
    cv::Mat image(height, width, CV_8UC3, map.data, cv::Mat::AUTO_STEP);
    cv::Mat image_copy = image.clone();
    
    // Rotate right camera image by 180 degrees
    if (camera == "right") {
      cv::rotate(image_copy, image_copy, cv::ROTATE_180);
    }
    
    CameraFrame frame;
    // Use buffer timestamp if available, otherwise current time
    GstClockTime timestamp = GST_BUFFER_PTS(buffer);
    if (timestamp == GST_CLOCK_TIME_NONE) {
      GstClock* clock = gst_element_get_clock(appsink);
      if (clock) {
        timestamp = gst_clock_get_time(clock);
        gst_object_unref(clock);
      } else {
        // Fallback to system time in nanoseconds
        timestamp = static_cast<GstClockTime>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
      }
    }
    frame.ts_ns = timestamp; // Already in nanoseconds
    frame.image = image_copy;
    frame.camera_label = camera;
    
    ring_.push(std::move(frame));
    
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return GST_FLOW_OK;
  }

  static gboolean bus_func(GstBus* /*bus*/, GstMessage* msg, gpointer user_data) {
    auto* self = static_cast<UnifiedDataCollector*>(user_data);
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        RCLCPP_ERROR(self->get_logger(), "GStreamer ERROR from %s: %s",
                     GST_OBJECT_NAME(msg->src), err ? err->message : "unknown");
        if (dbg) { RCLCPP_ERROR(self->get_logger(), "Debug: %s", dbg); g_free(dbg); }
        if (err) g_error_free(err);
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        RCLCPP_WARN(self->get_logger(), "GStreamer WARNING from %s: %s",
                    GST_OBJECT_NAME(msg->src), err ? err->message : "unknown");
        if (dbg) { RCLCPP_DEBUG(self->get_logger(), "Debug: %s", dbg); g_free(dbg); }
        if (err) g_error_free(err);
        break;
      }
      case GST_MESSAGE_STATE_CHANGED: {
        // Only log state changes for the pipeline itself, not child elements
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT(self->pipeline_.get())) {
          GstState old_state, new_state, pending_state;
          gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
          RCLCPP_DEBUG(self->get_logger(), "Pipeline state: %s -> %s",
                       gst_element_state_get_name(old_state),
                       gst_element_state_get_name(new_state));
        }
        break;
      }
      default:
        break;
    }
    return TRUE;
  }

  static std::string timestampStr(){
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm,&t);
#else
    localtime_r(&t,&tm);
#endif
    char buf[32]; std::strftime(buf,sizeof(buf),"%Y%m%d_%H%M%S",&tm);
    return std::string(buf);
  }

private:
  Config cfg_;
  std::mutex mu_;
  std::atomic<bool> recording_active_;
  std::atomic<bool> shutting_down_;
  std::chrono::steady_clock::time_point last_log_time_;
  
  // Camera streaming selection
  std::atomic<bool> streaming_right_camera_{false};  // false = left, true = right
  std::atomic<bool> camera_switching_{false};        // true during pipeline rebuild
  std::chrono::steady_clock::time_point camera_switch_start_;
  static constexpr int kCameraSwitchGracePeriodMs = 3000;  // Mark rows for 3s after switch

  FrameRing ring_;
  CsvWriter writer_;

  fs::path session_dir_;
  std::shared_ptr<std::ofstream> csv_stream_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_select_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stream_target_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stream_status_pub_;

  // Thermal Camera
  seekcamera_manager_t* mgr_ = nullptr;
  seekcamera_t* thermal_cam_ = nullptr;

  // GStreamer
  struct GstElementDeleter { void operator()(GstElement* p) const { if (p) gst_object_unref(p); } };
  std::unique_ptr<GstElement, GstElementDeleter> pipeline_;
  std::string pipeline_str_;
  GstElement* left_appsink_{nullptr};
  GstElement* right_appsink_{nullptr};
  GstBus* bus_{nullptr};
  guint bus_watch_id_{0};
  GMainContext* context_{nullptr};
  GMainLoop* loop_{nullptr};
  std::thread loop_thread_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  try { 
    auto node = std::make_shared<UnifiedDataCollector>();
    rclcpp::spin(node);
  }
  catch(const std::exception& e){ 
    std::cerr<<"Fatal: "<<e.what()<<"\n"; 
  }
  rclcpp::shutdown(); 
  return 0;
}
