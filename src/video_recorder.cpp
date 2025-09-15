#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

struct ResolutionOption {
  int width;
  int height;
};

class SingleCameraRecorder {
public:
  SingleCameraRecorder(const std::string &device_path,
                       const std::string &camera_label,
                       const std::string &output_directory,
                       const std::string &fourcc_str,
                       double desired_fps)
      : device_path_(device_path),
        camera_label_(camera_label),
        output_directory_(output_directory),
        fourcc_str_(fourcc_str),
        desired_fps_(desired_fps),
        running_(false) {}

  ~SingleCameraRecorder() { stop(); }

  bool start() {
    if (running_.load()) return true;

    // Create output directory
    std::error_code ec;
    fs::create_directories(output_directory_, ec);

    // Open capture
    if (!cap_.open(device_path_, cv::CAP_V4L2)) {
      last_error_ = "Failed to open camera: " + device_path_;
      return false;
    }

    // Reduce internal buffering to minimize latency
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Request camera output pixel format (e.g., MJPG or UYVY) if supported
    int cap_fourcc = cv::VideoWriter::fourcc(
        fourcc_str_.size() > 0 ? fourcc_str_[0] : 'M',
        fourcc_str_.size() > 1 ? fourcc_str_[1] : 'J',
        fourcc_str_.size() > 2 ? fourcc_str_[2] : 'P',
        fourcc_str_.size() > 3 ? fourcc_str_[3] : 'G');
    cap_.set(cv::CAP_PROP_FOURCC, cap_fourcc);

    // Try to set desired FPS if provided
    if (desired_fps_ > 0.0) {
      cap_.set(cv::CAP_PROP_FPS, desired_fps_);
    }

    // Attempt highest resolution from a list of common options (descending)
    const std::vector<ResolutionOption> candidates = {
        {4096, 2160}, {3840, 2160}, {2560, 1440}, {2048, 1536}, {1920, 1200},
        {1920, 1080}, {1600, 1200}, {1600, 900},  {1280, 1024}, {1280, 800},
        {1280, 720},  {1024, 768},  {800, 600},   {640, 480}};

    bool set_any = false;
    for (const auto &opt : candidates) {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, opt.width);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, opt.height);
      double got_w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
      double got_h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
      if (static_cast<int>(got_w + 0.5) == opt.width &&
          static_cast<int>(got_h + 0.5) == opt.height) {
        set_any = true;
        break;
      }
    }

    int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    if (actual_fps <= 0.0) {
      actual_fps = (desired_fps_ > 0.0) ? desired_fps_ : 30.0;
    }

    // Build output filename
    const auto now = std::chrono::system_clock::now();
    const std::time_t tnow = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &tnow);
#else
    localtime_r(&tnow, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");

    const std::string base_name = oss.str() + "_" + camera_label_;
    const std::string ext = ".avi";  // widely supported
    const std::string out_path = (fs::path(output_directory_) / (base_name + ext)).string();

    // Prepare writer
    int fourcc = cv::VideoWriter::fourcc(
        fourcc_str_.size() > 0 ? fourcc_str_[0] : 'M',
        fourcc_str_.size() > 1 ? fourcc_str_[1] : 'J',
        fourcc_str_.size() > 2 ? fourcc_str_[2] : 'P',
        fourcc_str_.size() > 3 ? fourcc_str_[3] : 'G');

    if (!writer_.open(out_path, fourcc, actual_fps, cv::Size(actual_w, actual_h))) {
      last_error_ = "Failed to open writer: " + out_path;
      cap_.release();
      return false;
    }

    output_path_ = out_path;
    running_.store(true);
    worker_ = std::thread([this]() { this->loop(); });
    return true;
  }

  void stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    if (writer_.isOpened()) writer_.release();
    if (cap_.isOpened()) cap_.release();
  }

  std::string output_path() const { return output_path_; }
  std::string last_error() const { return last_error_; }

private:
  void loop() {
    cv::Mat frame;
    while (running_.load()) {
      if (!cap_.read(frame)) {
        last_error_ = "Failed to read frame from " + device_path_;
        break;
      }
      if (!frame.empty()) {
        writer_.write(frame);
      }
    }
  }

  std::string device_path_;
  std::string camera_label_;
  std::string output_directory_;
  std::string fourcc_str_;
  double desired_fps_;

  std::atomic<bool> running_;
  std::thread worker_;
  cv::VideoCapture cap_;
  cv::VideoWriter writer_;
  std::string output_path_;
  std::string last_error_;
};

class DualVideoRecorderNode : public rclcpp::Node {
public:
  DualVideoRecorderNode()
      : rclcpp::Node("video_recorder"),
        recording_(false) {
    // Parameters
    this->declare_parameter<std::string>(
        "left_device", "auto");  // auto-discover /dev/v4l/by-id for index0
    this->declare_parameter<std::string>(
        "right_device", "auto");
    this->declare_parameter<std::string>(
        "output_dir", default_output_dir());
    this->declare_parameter<std::string>(
        "fourcc", "MJPG");
    this->declare_parameter<double>(
        "fps", 30.0);

    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/video_record_set",
        std::bind(&DualVideoRecorderNode::onSetRecording, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "video_recorder node ready. Call /video_record_set to control.");
  }

private:
  static std::string default_output_dir() {
    const char *home = std::getenv("HOME");
    if (home && std::string(home).size() > 0) {
      return std::string(home) + "/scan_videos";
    }
    return std::string("/home/user/scan_videos");
  }

  static std::vector<std::string> discover_index0_devices() {
    std::vector<std::string> results;
    const fs::path by_id("/dev/v4l/by-id");
    std::error_code ec;
    if (!fs::exists(by_id, ec)) return results;
    for (const auto &entry : fs::directory_iterator(by_id, ec)) {
      if (!entry.is_symlink(ec) && !entry.is_regular_file(ec)) continue;
      const std::string name = entry.path().filename().string();
      if (name.find("See3CAM") != std::string::npos &&
          name.find("video-index0") != std::string::npos) {
        results.emplace_back(entry.path().string());
      }
    }
    std::sort(results.begin(), results.end());
    return results;
  }

  void onSetRecording(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    std::lock_guard<std::mutex> lock(mu_);
    if (req->data) {
      if (recording_) {
        resp->success = true;
        resp->message = "Already recording";
        return;
      }
      // Read parameters
      std::string left_dev = this->get_parameter("left_device").as_string();
      std::string right_dev = this->get_parameter("right_device").as_string();
      std::string output_dir = this->get_parameter("output_dir").as_string();
      std::string fourcc = this->get_parameter("fourcc").as_string();
      double fps = this->get_parameter("fps").as_double();

      if (left_dev == "auto" || right_dev == "auto") {
        auto devices = discover_index0_devices();
        if (devices.size() < 2) {
          resp->success = false;
          resp->message = "Auto-discover failed: need 2 See3CAM index0 devices";
          RCLCPP_ERROR(this->get_logger(), "%s", resp->message.c_str());
          return;
        }
        if (left_dev == "auto") left_dev = devices[0];
        if (right_dev == "auto") right_dev = devices[1];
      }

      // Labels from device paths for readability
      const std::string left_label = "cam_left";
      const std::string right_label = "cam_right";

      left_recorder_ = std::make_unique<SingleCameraRecorder>(left_dev, left_label, output_dir, fourcc, fps);
      right_recorder_ = std::make_unique<SingleCameraRecorder>(right_dev, right_label, output_dir, fourcc, fps);

      if (!left_recorder_->start()) {
        resp->success = false;
        resp->message = std::string("Left start failed: ") + left_recorder_->last_error();
        left_recorder_.reset();
        right_recorder_.reset();
        RCLCPP_ERROR(this->get_logger(), "%s", resp->message.c_str());
        return;
      }

      if (!right_recorder_->start()) {
        std::string err = right_recorder_->last_error();
        left_recorder_->stop();
        left_recorder_.reset();
        right_recorder_.reset();
        resp->success = false;
        resp->message = std::string("Right start failed: ") + err;
        RCLCPP_ERROR(this->get_logger(), "%s", resp->message.c_str());
        return;
      }

      recording_ = true;
      resp->success = true;
      resp->message = "Recording started";
      RCLCPP_INFO(this->get_logger(), "Recording started. Output dir: %s", output_dir.c_str());
    } else {
      if (!recording_) {
        resp->success = true;
        resp->message = "Already stopped";
        return;
      }

      if (left_recorder_) left_recorder_->stop();
      if (right_recorder_) right_recorder_->stop();
      left_recorder_.reset();
      right_recorder_.reset();
      recording_ = false;
      resp->success = true;
      resp->message = "Recording stopped";
      RCLCPP_INFO(this->get_logger(), "Recording stopped.");
    }
  }

  std::mutex mu_;
  bool recording_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::unique_ptr<SingleCameraRecorder> left_recorder_;
  std::unique_ptr<SingleCameraRecorder> right_recorder_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualVideoRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


