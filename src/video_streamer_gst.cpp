#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gst/gst.h>
#include <glib.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <condition_variable>
#include <filesystem>

class VideoStreamerGstNode : public rclcpp::Node {
public:
  VideoStreamerGstNode() : rclcpp::Node("video_streamer_gst"), recording_active_(false), shutting_down_(false) {
    // Parameters
    this->declare_parameter<std::string>("left_device", "/dev/v4l/by-id/See3CAM_Left-video-index0");
    this->declare_parameter<std::string>("right_device", "/dev/v4l/by-id/See3CAM_Right-video-index0");
    this->declare_parameter<std::string>("camera_label", "cam_left");
    this->declare_parameter<std::string>("camera_label_right", "cam_right");
    this->declare_parameter<std::string>("output_dir", default_output_dir());
    this->declare_parameter<std::string>("fourcc", "MJPG");
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<bool>("start_recording", false);
    this->declare_parameter<bool>("enable_record_service", false);

    this->declare_parameter<bool>("use_mjpeg_pipeline", true);
    this->declare_parameter<int>("cap_w", 1920);
    this->declare_parameter<int>("cap_h", 1080);
    this->declare_parameter<int>("cap_fps", 30);

    this->declare_parameter<std::string>("raw_format", "UYVY");
    this->declare_parameter<int>("raw_w", 1280);
    this->declare_parameter<int>("raw_h", 720);
    this->declare_parameter<int>("raw_fps", 60);

    this->declare_parameter<std::string>("stream_host", "172.16.10.121");
    this->declare_parameter<int>("stream_port", 5600);
    this->declare_parameter<int>("stream_bitrate_kbps", 800);
    this->declare_parameter<int>("rtp_mtu", 1200);

    // Service for recording control (compatible with existing teleop)
    if (this->get_parameter("enable_record_service").as_bool()) {
      record_srv_ = this->create_service<std_srvs::srv::SetBool>(
          "/video_record_set",
          std::bind(&VideoStreamerGstNode::onSetRecording, this, std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(this->get_logger(), "Service ready: /video_record_set (std_srvs/SetBool)");
    }

    // Initialize GStreamer (once per process)
    static std::once_flag gst_once;
    std::call_once(gst_once, []() { gst_init(nullptr, nullptr); });

    // Build and start pipeline
    build_pipeline();
    start_main_loop();

    // Optionally start recording immediately
    const bool start_rec = this->get_parameter("start_recording").as_bool();
    if (start_rec) {
      // Give the pipeline a brief moment to preroll
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      (void)start_recording();
    }
  }

  ~VideoStreamerGstNode() override {
    shutting_down_.store(true);
    stop_recording();
    stop_pipeline();
  }

private:
  // Helper to block until an element reaches target state (or timeout). Returns true on success.
  static bool set_state_blocking(GstElement *elem, GstState target, GstClockTime timeout_ns) {
    if (!elem) return false;
    gst_element_set_state(elem, target);
    GstState cur = GST_STATE_NULL, pending = GST_STATE_NULL;
    GstStateChangeReturn rc = gst_element_get_state(elem, &cur, &pending, timeout_ns);
    return rc != GST_STATE_CHANGE_FAILURE && cur == target;
  }
  static std::string default_output_dir() {
    const char *home = std::getenv("HOME");
    if (home && std::string(home).size() > 0) {
      return std::string(home) + "/scan_videos";
    }
    return std::string("/home/user/scan_videos");
  }

  // ===== Pipeline build/start/stop =====
  void build_pipeline() {
    const std::string device = this->get_parameter("left_device").as_string();
    const bool use_mjpeg = this->get_parameter("use_mjpeg_pipeline").as_bool();
    const int cap_w = this->get_parameter("cap_w").as_int();
    const int cap_h = this->get_parameter("cap_h").as_int();
    const int cap_fps = this->get_parameter("cap_fps").as_int();

    const std::string raw_format = this->get_parameter("raw_format").as_string();
    const int raw_w = this->get_parameter("raw_w").as_int();
    const int raw_h = this->get_parameter("raw_h").as_int();
    const int raw_fps = this->get_parameter("raw_fps").as_int();

    const std::string right_device = this->get_parameter("right_device").as_string();

    const double rec_fps_d = this->get_parameter("fps").as_double();
    const int rec_fps = static_cast<int>(rec_fps_d);

    const std::string stream_host = this->get_parameter("stream_host").as_string();
    const int stream_port = this->get_parameter("stream_port").as_int();
    const int stream_bitrate = this->get_parameter("stream_bitrate_kbps").as_int();
    const int rtp_mtu = this->get_parameter("rtp_mtu").as_int();

    std::ostringstream oss;
    // LEFT camera (stream + record)
    oss << (use_mjpeg
              ? "v4l2src device=" + device +
                    " ! image/jpeg,width=" + std::to_string(cap_w) + ",height=" + std::to_string(cap_h) + ",framerate=" + std::to_string(cap_fps) + "/1 "
                    "! jpegdec ! videoconvert ! video/x-raw,format=I420 "
              : "v4l2src device=" + device +
                    " ! video/x-raw,format=" + raw_format + ",width=" + std::to_string(raw_w) + ",height=" + std::to_string(raw_h) + ",framerate=" + std::to_string(raw_fps) + "/1 "
                    "! videoconvert ! video/x-raw,format=I420 ")
        << "! tee name=T ";

    // Record branch (non-leaky). Ensure encoded caps carry framerate for avimux; parse JPEG bitstream
    oss << " T. ! queue ! videorate ! video/x-raw,framerate=" << rec_fps << "/1 "
        << "! videoconvert ! video/x-raw,format=I420 "
        << "! valve name=valve_rec drop=true "
        << "! jpegenc quality=95 ! capssetter caps=\"image/jpeg,framerate=" << rec_fps << "/1\" ! jpegparse "
        << "! avimux name=avim ! filesink name=rec_sink async=false sync=false ";

    // Stream branch (leaky, exact chain requested)
    oss << " T. ! queue leaky=downstream max-size-buffers=120 max-size-bytes=0 max-size-time=0 "
        << "! videorate ! video/x-raw,framerate=15/1 "
        << "! videoscale ! video/x-raw,width=640,height=480 "
        << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << stream_bitrate << " key-int-max=30 bframes=0 "
        << "! video/x-h264,stream-format=byte-stream,alignment=au "
        << "! rtph264pay pt=96 config-interval=1 mtu=" << rtp_mtu << " "
        << "! udpsink host=" << stream_host << " port=" << stream_port << " sync=false";

    // RIGHT camera (record only)
    oss << " \n";
    if (use_mjpeg) {
      oss << "v4l2src device=" << right_device
          << " ! image/jpeg,width=" << cap_w << ",height=" << cap_h << ",framerate=" << cap_fps << "/1 "
          << "! jpegdec ! videoconvert ! video/x-raw,format=I420 ";
    } else {
      oss << "v4l2src device=" << right_device
          << " ! video/x-raw,format=" << raw_format << ",width=" << raw_w << ",height=" << raw_h << ",framerate=" << raw_fps << "/1 "
          << "! videoconvert ! video/x-raw,format=I420 ";
    }
    oss << "! queue ! videorate ! video/x-raw,framerate=" << rec_fps << "/1 "
        << "! videoconvert ! video/x-raw,format=I420 "
        << "! valve name=valve_rec_right drop=true "
        << "! jpegenc quality=95 ! capssetter caps=\"image/jpeg,framerate=" << rec_fps << "/1\" ! jpegparse "
        << "! avimux name=avim_right ! filesink name=rec_sink_right async=false sync=false ";

    const std::string pipeline_str = oss.str();
    RCLCPP_INFO(this->get_logger(), "Launching GStreamer pipeline:\n%s", pipeline_str.c_str());

    GError *err = nullptr;
    GstElement *pipe = gst_parse_launch(pipeline_str.c_str(), &err);
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

    // Grab elements we control
    valve_rec_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "valve_rec"));
    avim_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "avim"));
    rec_sink_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "rec_sink"));
    valve_rec_right_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "valve_rec_right"));
    avim_right_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "avim_right"));
    rec_sink_right_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "rec_sink_right"));

    if (!valve_rec_ || !avim_ || !rec_sink_ || !valve_rec_right_ || !avim_right_ || !rec_sink_right_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to retrieve required elements (left and right record chain)");
      throw std::runtime_error("Missing elements");
    }

    // Valves default to drop=true in pipeline string; ensure closed before any state changes
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);

    // Do not set PLAYING here; bus watch isn't attached yet. State is set in start_main_loop().
  }

  void start_main_loop() {
    // Create dedicated context and loop, attach bus watch to this context
    bus_ = gst_element_get_bus(pipeline_.get());
    if (!bus_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get pipeline bus");
      throw std::runtime_error("No bus");
    }

    context_ = g_main_context_new();
    loop_ = g_main_loop_new(context_, FALSE);

    loop_thread_ = std::thread([this]() {
      g_main_context_push_thread_default(context_);
      // Attach bus watch to this context (now default for this thread)
      bus_watch_id_ = gst_bus_add_watch(bus_, &VideoStreamerGstNode::bus_func, this);
      // Now that the bus watch is active, set pipeline to PLAYING so errors are reported
      GstStateChangeReturn ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(this->get_logger(), "Failed to set pipeline to PLAYING (see GStreamer logs above)");
      }
      g_main_loop_run(loop_);
      if (bus_watch_id_ != 0) {
        g_source_remove(bus_watch_id_);
        bus_watch_id_ = 0;
      }
      g_main_context_pop_thread_default(context_);
    });
  }

  void stop_pipeline() {
    if (pipeline_) {
      gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
    }
    if (loop_) {
      g_main_loop_quit(loop_);
      if (loop_thread_.joinable()) loop_thread_.join();
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
    if (valve_rec_) { gst_object_unref(valve_rec_); valve_rec_ = nullptr; }
    if (avim_) { gst_object_unref(avim_); avim_ = nullptr; }
    if (rec_sink_) { gst_object_unref(rec_sink_); rec_sink_ = nullptr; }
    pipeline_.reset();
  }

  // ===== Recording control =====
  void onSetRecording(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    RCLCPP_INFO(this->get_logger(), "onSetRecording request: %s (active=%s)", req->data ? "true" : "false", recording_active_ ? "true" : "false");
    if (req->data) {
      if (recording_active_) {
        resp->success = true;
        resp->message = "Already recording";
        return;
      }
      if (start_recording()) {
        resp->success = true;
        resp->message = "Recording started";
        RCLCPP_INFO(this->get_logger(), "Recording started OK");
      } else {
        resp->success = false;
        resp->message = "Failed to start recording";
        RCLCPP_ERROR(this->get_logger(), "Failed to start recording");
      }
    } else {
      if (!recording_active_) {
        resp->success = true;
        resp->message = "Already stopped";
        return;
      }
      if (stop_recording()) {
        resp->success = true;
        resp->message = "Recording stopped";
        RCLCPP_INFO(this->get_logger(), "Recording stopped OK");
      } else {
        resp->success = false;
        resp->message = "Failed to stop recording";
        RCLCPP_ERROR(this->get_logger(), "Failed to stop recording");
      }
    }
  }

  std::string build_output_path_left() const {
    const std::string output_dir = this->get_parameter("output_dir").as_string();
    const std::string label = this->get_parameter("camera_label").as_string();

    const auto now = std::chrono::system_clock::now();
    const std::time_t tnow = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &tnow);
#else
    localtime_r(&tnow, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << "_" << label << ".avi";
    std::string filename = oss.str();
    if (!output_dir.empty() && output_dir.back() == '/') {
      return output_dir + filename;
    }
    return output_dir + "/" + filename;
  }

  std::string build_output_path_right() const {
    const std::string output_dir = this->get_parameter("output_dir").as_string();
    const std::string label = this->get_parameter("camera_label_right").as_string();

    const auto now = std::chrono::system_clock::now();
    const std::time_t tnow = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &tnow);
#else
    localtime_r(&tnow, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << "_" << label << ".avi";
    std::string filename = oss.str();
    if (!output_dir.empty() && output_dir.back() == '/') {
      return output_dir + filename;
    }
    return output_dir + "/" + filename;
  }

  bool start_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;

    const std::string location_left = build_output_path_left();
    const std::string location_right = build_output_path_right();
    // Ensure directory exists
    try {
      std::filesystem::create_directories(std::filesystem::path(location_left).parent_path());
      std::filesystem::create_directories(std::filesystem::path(location_right).parent_path());
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Failed to ensure output directories exist");
    }

    // Ensure no buffers flow
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);

    // Fully close sinks/muxers before changing filesink locations
    (void)set_state_blocking(avim_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(rec_sink_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(avim_right_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_NULL, GST_SECOND);

    // Set target file locations now that sinks are not open
    g_object_set(G_OBJECT(rec_sink_), "location", location_left.c_str(), NULL);
    g_object_set(G_OBJECT(rec_sink_right_), "location", location_right.c_str(), NULL);
    RCLCPP_INFO(this->get_logger(), "Recording LEFT to: %s", location_left.c_str());
    RCLCPP_INFO(this->get_logger(), "Recording RIGHT to: %s", location_right.c_str());

    // Bring branches back up synchronously: NULL->READY->PAUSED->PLAYING
    (void)set_state_blocking(rec_sink_, GST_STATE_READY, GST_SECOND);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_READY, GST_SECOND);
    (void)set_state_blocking(avim_, GST_STATE_READY, GST_SECOND);
    (void)set_state_blocking(avim_right_, GST_STATE_READY, GST_SECOND);
    (void)set_state_blocking(rec_sink_, GST_STATE_PAUSED, GST_SECOND);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_PAUSED, GST_SECOND);
    (void)set_state_blocking(avim_, GST_STATE_PAUSED, GST_SECOND);
    (void)set_state_blocking(avim_right_, GST_STATE_PAUSED, GST_SECOND);
    (void)set_state_blocking(rec_sink_, GST_STATE_PLAYING, GST_SECOND);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_PLAYING, GST_SECOND);
    (void)set_state_blocking(avim_, GST_STATE_PLAYING, GST_SECOND);
    (void)set_state_blocking(avim_right_, GST_STATE_PLAYING, GST_SECOND);

    // Open valve
    g_object_set(G_OBJECT(valve_rec_), "drop", FALSE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", FALSE, NULL);
    recording_active_ = true;
    return true;
  }

  bool stop_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;
    if (!recording_active_) return true;

    // Close valve to stop new buffers
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);

    // Send EOS to finalize AVI cleanly (send to muxers and sinks for robustness)
    eos_left_received_ = false;
    eos_right_received_ = false;
    if (!gst_element_send_event(avim_, gst_event_new_eos())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send EOS to avim branch");
    }
    if (!gst_element_send_event(avim_right_, gst_event_new_eos())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send EOS to avim_right branch");
    }
    if (!gst_element_send_event(rec_sink_, gst_event_new_eos())) {
      RCLCPP_DEBUG(this->get_logger(), "filesink left EOS send returned false (may be ok)");
    }
    if (!gst_element_send_event(rec_sink_right_, gst_event_new_eos())) {
      RCLCPP_DEBUG(this->get_logger(), "filesink right EOS send returned false (may be ok)");
    }

    // Wait for EOS confirmation for a bounded time
    {
      std::unique_lock<std::mutex> ul(cv_mu_);
      cv_.wait_for(ul, std::chrono::seconds(5), [this]() { return eos_left_received_.load() && eos_right_received_.load(); });
    }

    // Keep valves closed and fully close sinks/muxers so files are not kept open
    (void)set_state_blocking(avim_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(rec_sink_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(avim_right_, GST_STATE_NULL, GST_SECOND);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_NULL, GST_SECOND);

    recording_active_ = false;
    return true;
  }

  // ===== Bus watch =====
  static gboolean bus_source_func(GstBus * /*bus*/, GstMessage *msg, gpointer user_data) {
    // Deprecated path; keep for compatibility if needed
    auto *self = static_cast<VideoStreamerGstNode *>(user_data);
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        RCLCPP_ERROR(self->get_logger(), "GStreamer ERROR: %s", err ? err->message : "unknown");
        if (dbg) g_free(dbg);
        if (err) g_error_free(err);
        break;
      }
      case GST_MESSAGE_EOS: {
        // Identify EOS from record branches (left/right)
        GstObject *src = GST_MESSAGE_SRC(msg);
        {
          std::lock_guard<std::mutex> lk(self->cv_mu_);
          // Mark left/right as received if message originates from either branch (sink or avim or pipeline)
          if (self->avim_ && (src == GST_OBJECT(self->avim_))) self->eos_left_received_.store(true);
          if (self->rec_sink_ && (src == GST_OBJECT(self->rec_sink_))) self->eos_left_received_.store(true);
          if (self->avim_right_ && (src == GST_OBJECT(self->avim_right_))) self->eos_right_received_.store(true);
          if (self->rec_sink_right_ && (src == GST_OBJECT(self->rec_sink_right_))) self->eos_right_received_.store(true);
          // As a fallback, if pipeline posts EOS, consider both done
          if (GST_MESSAGE_SRC(msg) == GST_OBJECT(self->pipeline_.get())) {
            self->eos_left_received_.store(true);
            self->eos_right_received_.store(true);
          }
          self->cv_.notify_all();
        }
        break;
      }
      default:
        break;
    }
    return TRUE;
  }

  static gboolean bus_func(GstBus * /*bus*/, GstMessage *msg, gpointer user_data) {
    return bus_source_func(nullptr, msg, user_data);
  }

  // ===== Members =====
  std::mutex mu_;
  std::atomic<bool> recording_active_;
  std::atomic<bool> shutting_down_;

  // EOS sync for recording branch
  std::mutex cv_mu_;
  std::condition_variable cv_;
  std::atomic<bool> eos_left_received_{false};
  std::atomic<bool> eos_right_received_{false};

  // GStreamer bits
  struct GstElementDeleter { void operator()(GstElement *p) const { if (p) gst_object_unref(p); } };
  std::unique_ptr<GstElement, GstElementDeleter> pipeline_;
  GstElement *valve_rec_ {nullptr};
  GstElement *avim_ {nullptr};
  GstElement *rec_sink_ {nullptr};
  GstElement *valve_rec_right_ {nullptr};
  GstElement *avim_right_ {nullptr};
  GstElement *rec_sink_right_ {nullptr};
  GstBus *bus_ {nullptr};
  guint bus_watch_id_ {0};
  GMainContext *context_ {nullptr};
  GMainLoop *loop_ {nullptr};
  std::thread loop_thread_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoStreamerGstNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


