#include <seekcamera/seekcamera.h>
#include <seekcamera/seekcamera_manager.h>
#include <seekcamera/seekcamera_frame.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  std::string odom_topic      = "/Odometry";
  std::string log_directory   = std::string(getenv("HOME")?getenv("HOME"):".") + "/gpr_scans";
  bool use_seekvision_mode    = true;   // SeekVision pipeline for colorized stream
  bool save_color_png         = true;   // write *_color.png alongside float32 bin
  int  csv_flush_every_rows   = 50;     // batch CSV flush, no per-row flush
  size_t frame_ring_size      = 24;     // ~2–3 s @ 9 Hz camera
};

// ==================== Thermal frame buffer ====================
struct ThermFrame {
  uint64_t ts_ns = 0;  // SDK UTC timestamp (ns)
  int w = 0, h = 0;
  std::vector<float> thermo;   // °C, size w*h
  cv::Mat color_bgr;           // colorized display (optional)
};

class FrameRing {
public:
  explicit FrameRing(size_t cap) : cap_(cap) {}
  void push(ThermFrame&& f) {
    std::lock_guard<std::mutex> lk(m_);
    q_.emplace_back(std::move(f));
    while (q_.size() > cap_) q_.pop_front();
  }
  bool nearest(uint64_t target_ns, ThermFrame& out, uint64_t& dt_abs_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return false;
    size_t best = 0; uint64_t bestdt = UINT64_MAX;
    for (size_t i=0;i<q_.size();++i) {
      uint64_t a=q_[i].ts_ns,b=target_ns; uint64_t d=(a>b)?(a-b):(b-a);
      if (d<bestdt){ bestdt=d; best=i; }
    }
    out = q_[best]; dt_abs_ns = bestdt; return true;
  }
private:
  std::mutex m_;
  std::deque<ThermFrame> q_;
  size_t cap_;
};

// ==================== Async writer ====================
struct CsvJob {
  uint64_t odom_ns{}, cam_ns{}; double dt_ms{};
  // pose / orientation
  double px{},py{},pz{}, qx{},qy{},qz{},qw{};
  // twist
  double vx{},vy{},vz{}, wx{},wy{},wz{};
  // data
  ThermFrame frame;
  fs::path out_dir;                            // session_dir/frames
  std::shared_ptr<std::ofstream> csv;          // session_dir/thermal_log.csv
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
    fs::path p = dir / (stem + "_thermo_f32.bin");
    std::ofstream ofs(p, std::ios::binary);
    int32_t w=f.w,h=f.h;
    ofs.write(reinterpret_cast<const char*>(&w),4);
    ofs.write(reinterpret_cast<const char*>(&h),4);
    ofs.write(reinterpret_cast<const char*>(f.thermo.data()), f.w*f.h*sizeof(float));
    return p.string();
  }
  static std::string save_color_png(const ThermFrame& f, const fs::path& dir, const std::string& stem) {
    if (f.color_bgr.empty()) return "";
    fs::create_directories(dir);
    fs::path p = dir / (stem + "_color.png");
    cv::imwrite(p.string(), f.color_bgr);
    return p.string();
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
      std::ostringstream s; s << "odom" << job.odom_ns << "_cam" << job.cam_ns;
      const std::string stem = s.str();

      const std::string thermo_bin = save_float32_bin(job.frame, job.out_dir, stem);
      std::string color_png;
      if (!job.frame.color_bgr.empty()) color_png = save_color_png(job.frame, job.out_dir, stem);

      auto& csv = *job.csv;
      csv << job.odom_ns << "," << job.cam_ns << "," << std::fixed << std::setprecision(3) << job.dt_ms << ","
          << job.px << "," << job.py << "," << job.pz << ","
          << job.qx << "," << job.qy << "," << job.qz << "," << job.qw << ","
          << job.vx << "," << job.vy << "," << job.vz << ","
          << job.wx << "," << job.wy << "," << job.wz << ","
          << thermo_bin << "," << color_png << "\n";

      if (++rows_since_flush >= flush_every_rows_) { csv.flush(); rows_since_flush = 0; }
    }
  }

  std::thread th_;
  std::mutex m_; std::condition_variable cv_; std::deque<CsvJob> q_;
  std::atomic<bool> running_{false};
  int flush_every_rows_;
};

// ==================== Node ====================
class ThermalSyncLogger : public rclcpp::Node {
public:
  ThermalSyncLogger()
  : Node("thermal_sync_logger"),
    ring_(cfg_.frame_ring_size),
    writer_(cfg_.csv_flush_every_rows)
  {
    // Params
    declare_parameter("fastlio_odom_topic",   cfg_.odom_topic);
    declare_parameter("log_directory",        cfg_.log_directory);
    declare_parameter("use_seekvision_mode",  cfg_.use_seekvision_mode);
    declare_parameter("save_color_png",       cfg_.save_color_png);
    declare_parameter("csv_flush_every_rows", cfg_.csv_flush_every_rows);

    get_parameter("fastlio_odom_topic",   cfg_.odom_topic);
    get_parameter("log_directory",        cfg_.log_directory);
    get_parameter("use_seekvision_mode",  cfg_.use_seekvision_mode);
    get_parameter("save_color_png",       cfg_.save_color_png);
    get_parameter("csv_flush_every_rows", cfg_.csv_flush_every_rows);

    // Session dir
    session_dir_ = fs::path(cfg_.log_directory) / ("dataset_" + timestampStr());
    fs::create_directories(session_dir_ / "frames");

    // CSV open once
    csv_stream_ = std::make_shared<std::ofstream>((session_dir_ / "thermal_log.csv").string(),
                                                  std::ios::out | std::ios::trunc);
    *csv_stream_ << "odom_stamp_ns,cam_stamp_ns,dt_ms,"
                    "px,py,pz,qx,qy,qz,qw,"
                    "vx,vy,vz,wx,wy,wz,"
                    "thermo_f32_bin,color_png\n";

    // ROS wiring
    auto qos = rclcpp::SensorDataQoS().keep_last(100);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(cfg_.odom_topic, qos,
                 std::bind(&ThermalSyncLogger::onOdom, this, std::placeholders::_1));

    // Camera + writer
    createCameraManager();
    writer_.start();

    RCLCPP_INFO(get_logger(), "ThermalSyncLogger ready. Odom: %s", cfg_.odom_topic.c_str());
    RCLCPP_INFO(get_logger(), "Session: %s", session_dir_.c_str());
  }

  ~ThermalSyncLogger() override {
    writer_.stop();
    if (csv_stream_ && csv_stream_->is_open()) { csv_stream_->flush(); csv_stream_->close(); }
    destroyCameraManager();
  }

private:
  // ---------- Odom -> enqueue one row ----------
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const uint64_t odom_ns = (uint64_t)msg->header.stamp.sec*1000000000ULL
                           + (uint64_t)msg->header.stamp.nanosec;

    ThermFrame f; uint64_t dt=0;
    if (!ring_.nearest(odom_ns, f, dt)) return;

    CsvJob job;
    job.odom_ns = odom_ns;
    job.cam_ns  = f.ts_ns;
    job.dt_ms   = (double)dt / 1e6;
    job.frame   = std::move(f);
    job.out_dir = session_dir_ / "frames";
    job.csv     = csv_stream_;

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

    writer_.enqueue(std::move(job));
  }

  // ---------- Camera plumbing ----------
  void createCameraManager() {
    if (seekcamera_manager_create(&mgr_, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS)
      throw std::runtime_error("Failed to create seekcamera manager");
    seekcamera_manager_register_event_callback(mgr_, &ThermalSyncLogger::onEventStatic, this);
  }
  void destroyCameraManager() {
    if (cam_) seekcamera_capture_session_stop(cam_);
    if (mgr_) seekcamera_manager_destroy(&mgr_);
    cam_ = nullptr; mgr_ = nullptr;
  }

  static void onEventStatic(seekcamera_t* cam, seekcamera_manager_event_t ev, seekcamera_error_t status, void* user){
    static_cast<ThermalSyncLogger*>(user)->onEvent(cam, ev, status);
  }
  void onEvent(seekcamera_t* cam, seekcamera_manager_event_t ev, seekcamera_error_t status){
    (void)status;
    if (ev == SEEKCAMERA_MANAGER_EVENT_CONNECT) {
      cam_ = cam;
      // Pipeline mode for display
      seekcamera_set_pipeline_mode(cam_, cfg_.use_seekvision_mode
                                        ? SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION
                                        : SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY);

      // Request: Thermography Float32 + (optional) color ARGB8888
      uint32_t fmts = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
      if (cfg_.save_color_png) fmts |= SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888;

      seekcamera_register_frame_available_callback(cam_, &ThermalSyncLogger::onFrameStatic, this);
      auto err = seekcamera_capture_session_start(cam_, fmts);
      if (err == SEEKCAMERA_SUCCESS) RCLCPP_INFO(get_logger(), "[Seek] Streaming.");
      else RCLCPP_ERROR(get_logger(), "[Seek] capture start failed: %d", (int)err);
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_DISCONNECT) {
      RCLCPP_WARN(get_logger(), "[Seek] Disconnected.");
      if (cam_) seekcamera_capture_session_stop(cam_);
      cam_ = nullptr;
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR) {
      // For Micro cores; harmless if not raised on Mosaic
      seekcamera_store_calibration_data(cam, nullptr, nullptr, nullptr);
    }
  }

  static void onFrameStatic(seekcamera_t* cam, seekcamera_frame_t* cam_frame, void* user){
    static_cast<ThermalSyncLogger*>(user)->onFrame(cam, cam_frame);
  }
  void onFrame(seekcamera_t* cam, seekcamera_frame_t* cam_frame){
    (void)cam;
    seekcamera_frame_lock(cam_frame);

    ThermFrame f;

    // Thermography float32 (°C)
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

    // Color ARGB (→ BGR) if enabled
    if (cfg_.save_color_png) {
      seekframe_t* argb=nullptr;
      if (seekcamera_frame_get_frame_by_format(cam_frame,
          SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888, &argb) == SEEKCAMERA_SUCCESS && argb) {
        int w=(int)seekframe_get_width(argb), h=(int)seekframe_get_height(argb);
        const uint8_t* p = static_cast<const uint8_t*>(seekframe_get_data(argb));
        cv::Mat argb32(h,w,CV_8UC4,const_cast<uint8_t*>(p));
        std::vector<cv::Mat> ch; ch.reserve(4);
        cv::split(argb32, ch);                    // A,R,G,B
        cv::Mat bgra; cv::merge({ch[3],ch[2],ch[1],ch[0]}, bgra); // B,G,R,A
        cv::cvtColor(bgra, f.color_bgr, cv::COLOR_BGRA2BGR);
      }
    }

    seekcamera_frame_unlock(cam_frame);
    if (!f.thermo.empty()) ring_.push(std::move(f));
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
  FrameRing ring_;
  CsvWriter writer_;

  fs::path session_dir_;
  std::shared_ptr<std::ofstream> csv_stream_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Camera
  seekcamera_manager_t* mgr_ = nullptr;
  seekcamera_t* cam_ = nullptr;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<ThermalSyncLogger>()); }
  catch(const std::exception& e){ std::cerr<<"Fatal: "<<e.what()<<"\n"; }
  rclcpp::shutdown(); return 0;
}
