#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gst/gst.h>
#include <glib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

class VideoStreamerGstNode : public rclcpp::Node {
public:
  VideoStreamerGstNode()
      : rclcpp::Node("video_streamer_gst"),
        recording_active_(false),
        shutting_down_(false) {
    // ---- Parameters (kept compatible with your prior recorder) ----
    this->declare_parameter<std::string>("left_device", "/dev/v4l/by-id/See3CAM_Left-video-index0");
    this->declare_parameter<std::string>("right_device", "/dev/v4l/by-id/See3CAM_Right-video-index0");
    this->declare_parameter<std::string>("camera_label", "cam_left");
    this->declare_parameter<std::string>("camera_label_right", "cam_right");
    this->declare_parameter<std::string>("output_dir", default_output_dir());
    this->declare_parameter<std::string>("fourcc", "MJPG");       // parity marker; recording is MJPG
    this->declare_parameter<double>("fps", 30.0);                 // target record fps
    this->declare_parameter<bool>("start_recording", false);
    this->declare_parameter<bool>("enable_record_service", false);

    // Capture mode
    this->declare_parameter<bool>("use_mjpeg_pipeline", true);
    this->declare_parameter<int>("cap_w", 1920);
    this->declare_parameter<int>("cap_h", 1080);
    this->declare_parameter<int>("cap_fps", 30);

    this->declare_parameter<std::string>("raw_format", "UYVY");
    this->declare_parameter<int>("raw_w", 1280);
    this->declare_parameter<int>("raw_h", 720);
    this->declare_parameter<int>("raw_fps", 60);

    // Streaming
    this->declare_parameter<std::string>("stream_host", "172.16.10.121");
    this->declare_parameter<int>("stream_port", 5600);
    this->declare_parameter<int>("stream_bitrate_kbps", 800);
    this->declare_parameter<int>("rtp_mtu", 1200);

    if (this->get_parameter("enable_record_service").as_bool()) {
      record_srv_ = this->create_service<std_srvs::srv::SetBool>(
          "/video_record_set",
          std::bind(&VideoStreamerGstNode::onSetRecording, this, std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(this->get_logger(), "Service ready: /video_record_set (std_srvs/SetBool)");
    }

    // Init GStreamer once
    static std::once_flag gst_once;
    std::call_once(gst_once, [] { gst_init(nullptr, nullptr); });

    build_pipeline();
    start_main_loop();

    if (this->get_parameter("start_recording").as_bool()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      (void)start_recording();
    }
  }

  ~VideoStreamerGstNode() override {
    shutting_down_.store(true);
    (void)stop_recording();
    stop_pipeline();
  }

private:
  // ---------- Utilities ----------
  static std::string default_output_dir() {
    const char* home = std::getenv("HOME");
    if (home && std::string(home).size() > 0) return std::string(home) + "/scan_videos";
    return "/home/user/scan_videos";
  }

  static bool set_state_blocking(GstElement* elem, GstState target, GstClockTime timeout_ns = 5 * GST_SECOND) {
    if (!elem) return false;
    gst_element_set_state(elem, target);
    GstState cur = GST_STATE_NULL, pending = GST_STATE_NULL;
    GstStateChangeReturn rc = gst_element_get_state(elem, &cur, &pending, timeout_ns);
    return rc != GST_STATE_CHANGE_FAILURE && cur == target;
  }

  std::string build_output_path(const std::string& label) const {
    const std::string output_dir = this->get_parameter("output_dir").as_string();
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
    const std::string filename = oss.str();
    if (!output_dir.empty() && output_dir.back() == '/') return output_dir + filename;
    return output_dir + "/" + filename;
  }

  // ---------- Pipeline build ----------
  void build_pipeline() {
    const std::string left_dev   = this->get_parameter("left_device").as_string();
    const std::string right_dev  = this->get_parameter("right_device").as_string();
    const bool use_mjpeg         = this->get_parameter("use_mjpeg_pipeline").as_bool();
    const int cap_w              = this->get_parameter("cap_w").as_int();
    const int cap_h              = this->get_parameter("cap_h").as_int();
    const int cap_fps            = this->get_parameter("cap_fps").as_int();
    const std::string raw_format = this->get_parameter("raw_format").as_string();
    const int raw_w              = this->get_parameter("raw_w").as_int();
    const int raw_h              = this->get_parameter("raw_h").as_int();
    const int raw_fps            = this->get_parameter("raw_fps").as_int();

    const double rec_fps_d = this->get_parameter("fps").as_double();
    const int rec_fps      = static_cast<int>(rec_fps_d);

    const std::string stream_host = this->get_parameter("stream_host").as_string();
    const int stream_port         = this->get_parameter("stream_port").as_int();
    const int stream_bitrate      = this->get_parameter("stream_bitrate_kbps").as_int();
    const int rtp_mtu             = this->get_parameter("rtp_mtu").as_int();

    std::ostringstream oss;

    // ---- LEFT camera (stream + record) ----
    if (use_mjpeg) {
      oss << "v4l2src device=" << left_dev << " do-timestamp=true "
          << "! image/jpeg,width=" << cap_w << ",height=" << cap_h << ",framerate=" << cap_fps << "/1 "
          << "! jpegdec ! videoconvert ! video/x-raw,format=I420 ";
    } else {
      oss << "v4l2src device=" << left_dev << " do-timestamp=true "
          << "! video/x-raw,format=" << raw_format << ",width=" << raw_w << ",height=" << raw_h << ",framerate=" << raw_fps << "/1 "
          << "! videoconvert ! video/x-raw,format=I420 ";
    }
    oss << "! tee name=T ";

    // Record branch (non-leaky) — simplified & robust
    oss << " T. ! queue "
        << "! videorate ! video/x-raw,framerate=" << rec_fps << "/1 "
        << "! videoconvert ! video/x-raw,format=I420 "
        << "! valve name=valve_rec drop=true "
        << "! jpegenc quality=95 "
        << "! avimux name=avim "
        << "! queue "
        << "! filesink name=rec_sink async=false sync=false ";

    // Stream branch (leaky) — exact chain requested
    oss << " T. ! queue leaky=downstream max-size-buffers=120 max-size-bytes=0 max-size-time=0 "
        << "! videorate ! video/x-raw,framerate=15/1 "
        << "! videoscale ! video/x-raw,width=640,height=480 "
        << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << stream_bitrate << " key-int-max=30 bframes=0 "
        << "! video/x-h264,stream-format=byte-stream,alignment=au "
        << "! rtph264pay pt=96 config-interval=1 mtu=" << rtp_mtu << " "
        << "! udpsink host=" << stream_host << " port=" << stream_port << " sync=false ";

    // ---- RIGHT camera (record only) — mirrored clean branch ----
    if (use_mjpeg) {
      oss << "\n"
          << "v4l2src device=" << right_dev << " do-timestamp=true "
          << "! image/jpeg,width=" << cap_w << ",height=" << cap_h << ",framerate=" << cap_fps << "/1 "
          << "! jpegdec ! videoconvert ! video/x-raw,format=I420 ";
    } else {
      oss << "\n"
          << "v4l2src device=" << right_dev << " do-timestamp=true "
          << "! video/x-raw,format=" << raw_format << ",width=" << raw_w << ",height=" << raw_h << ",framerate=" << raw_fps << "/1 "
          << "! videoconvert ! video/x-raw,format=I420 ";
    }
    oss << "! queue "
        << "! videorate ! video/x-raw,framerate=" << rec_fps << "/1 "
        << "! videoconvert ! video/x-raw,format=I420 "
        << "! valve name=valve_rec_right drop=true "
        << "! jpegenc quality=95 "
        << "! avimux name=avim_right "
        << "! queue "
        << "! filesink name=rec_sink_right async=false sync=false ";

    pipeline_str_ = oss.str();
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

    // Cache elements we control
    valve_rec_       = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "valve_rec"));
    avim_            = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "avim"));
    rec_sink_        = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "rec_sink"));
    valve_rec_right_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "valve_rec_right"));
    avim_right_      = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "avim_right"));
    rec_sink_right_  = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "rec_sink_right"));

    if (!valve_rec_ || !avim_ || !rec_sink_ || !valve_rec_right_ || !avim_right_ || !rec_sink_right_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to retrieve required elements");
      throw std::runtime_error("Missing elements");
    }

    // Ensure valves closed before any state movement
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);
  }

  // ---------- GLib main loop ----------
  void start_main_loop() {
    bus_ = gst_element_get_bus(pipeline_.get());
    if (!bus_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get pipeline bus");
      throw std::runtime_error("No bus");
    }

    context_ = g_main_context_new();
    loop_    = g_main_loop_new(context_, FALSE);

    loop_thread_ = std::thread([this]() {
      g_main_context_push_thread_default(context_);
      bus_watch_id_ = gst_bus_add_watch(bus_, &VideoStreamerGstNode::bus_func, this);
      auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(this->get_logger(), "Failed to set pipeline to PLAYING");
      }
      g_main_loop_run(loop_);
      if (bus_watch_id_) {
        g_source_remove(bus_watch_id_);
        bus_watch_id_ = 0;
      }
      g_main_context_pop_thread_default(context_);
    });
  }

  void stop_pipeline() {
    if (pipeline_) gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
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
    if (avim_)      { gst_object_unref(avim_);      avim_      = nullptr; }
    if (rec_sink_)  { gst_object_unref(rec_sink_);  rec_sink_  = nullptr; }
    if (valve_rec_right_) { gst_object_unref(valve_rec_right_); valve_rec_right_ = nullptr; }
    if (avim_right_)      { gst_object_unref(avim_right_);      avim_right_      = nullptr; }
    if (rec_sink_right_)  { gst_object_unref(rec_sink_right_);  rec_sink_right_  = nullptr; }
    pipeline_.reset();
  }

  // ---------- Recording control ----------
  void onSetRecording(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    if (req->data) {
      if (recording_active_) {
        resp->success = true;
        resp->message = "Already recording";
        return;
      }
      if (start_recording()) {
        resp->success = true;
        resp->message = "Recording started";
      } else {
        resp->success = false;
        resp->message = "Failed to start recording";
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
      } else {
        resp->success = false;
        resp->message = "Failed to stop recording";
      }
    }
  }

  bool start_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;

    const std::string left_path  = build_output_path(this->get_parameter("camera_label").as_string());
    const std::string right_path = build_output_path(this->get_parameter("camera_label_right").as_string());
    try {
      std::filesystem::create_directories(std::filesystem::path(left_path).parent_path());
      std::filesystem::create_directories(std::filesystem::path(right_path).parent_path());
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Failed to ensure output directories exist");
    }

    // Keep valves closed while (re)arming sinks/muxers and changing filenames
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);

    // Bring sinks/muxers to READY, set new locations, then PLAYING (avoid PLAYING->property changes)
    (void)set_state_blocking(rec_sink_, GST_STATE_READY);
    (void)set_state_blocking(avim_,     GST_STATE_READY);
    g_object_set(G_OBJECT(rec_sink_), "location", left_path.c_str(), NULL);

    (void)set_state_blocking(rec_sink_right_, GST_STATE_READY);
    (void)set_state_blocking(avim_right_,     GST_STATE_READY);
    g_object_set(G_OBJECT(rec_sink_right_), "location", right_path.c_str(), NULL);

    RCLCPP_INFO(this->get_logger(), "Recording LEFT  to: %s", left_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Recording RIGHT to: %s", right_path.c_str());

    // Move to PLAYING
    (void)set_state_blocking(avim_,     GST_STATE_PLAYING);
    (void)set_state_blocking(rec_sink_, GST_STATE_PLAYING);
    (void)set_state_blocking(avim_right_,     GST_STATE_PLAYING);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_PLAYING);

    // Open valves to start writing
    g_object_set(G_OBJECT(valve_rec_), "drop", FALSE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", FALSE, NULL);

    recording_active_ = true;
    return true;
  }

  bool stop_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;
    if (!recording_active_) return true;

    // Stop feeding new buffers
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);
    g_object_set(G_OBJECT(valve_rec_right_), "drop", TRUE, NULL);

    // Reset EOS flags
    eos_left_received_.store(false);
    eos_right_received_.store(false);

    // Send EOS to muxers only (let EOS flow downstream to filesinks to finalize AVI)
    (void)gst_element_send_event(avim_, gst_event_new_eos());
    (void)gst_element_send_event(avim_right_, gst_event_new_eos());

    // Wait for EOS *from filesinks* (they post EOS after writing the index)
    {
      std::unique_lock<std::mutex> ul(cv_mu_);
      cv_.wait_for(ul, std::chrono::seconds(5), [this]() {
        return eos_left_received_.load() && eos_right_received_.load();
      });
    }

    // After EOS handled, leave branches at READY (quickly re-armed on next start)
    (void)set_state_blocking(rec_sink_, GST_STATE_READY);
    (void)set_state_blocking(avim_,     GST_STATE_READY);
    (void)set_state_blocking(rec_sink_right_, GST_STATE_READY);
    (void)set_state_blocking(avim_right_,     GST_STATE_READY);

    recording_active_ = false;
    return true;
  }

  // ---------- Bus watch ----------
  static gboolean bus_func(GstBus* /*bus*/, GstMessage* msg, gpointer user_data) {
    auto* self = static_cast<VideoStreamerGstNode*>(user_data);
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
      case GST_MESSAGE_EOS: {
        GstObject* src = GST_MESSAGE_SRC(msg);
        const char* name = GST_OBJECT_NAME(src);
        // We only mark completion when *filesinks* report EOS (muxers may post earlier)
        bool notify = false;
        {
          std::lock_guard<std::mutex> lk(self->cv_mu_);
          if (self->rec_sink_ && src == GST_OBJECT(self->rec_sink_)) {
            self->eos_left_received_.store(true);
            notify = true;
          }
          if (self->rec_sink_right_ && src == GST_OBJECT(self->rec_sink_right_)) {
            self->eos_right_received_.store(true);
            notify = true;
          }
        }
        if (notify) self->cv_.notify_all();
        RCLCPP_DEBUG(self->get_logger(), "EOS from %s", name ? name : "<unknown>");
        break;
      }
      default:
        break;
    }
    return TRUE;
  }

  // ---------- Members ----------
  std::mutex mu_;
  std::atomic<bool> recording_active_;
  std::atomic<bool> shutting_down_;

  // EOS synchronization
  std::mutex cv_mu_;
  std::condition_variable cv_;
  std::atomic<bool> eos_left_received_{false};
  std::atomic<bool> eos_right_received_{false};

  // GStreamer
  struct GstElementDeleter { void operator()(GstElement* p) const { if (p) gst_object_unref(p); } };
  std::unique_ptr<GstElement, GstElementDeleter> pipeline_;
  std::string pipeline_str_;

  GstElement* valve_rec_{nullptr};
  GstElement* avim_{nullptr};
  GstElement* rec_sink_{nullptr};

  GstElement* valve_rec_right_{nullptr};
  GstElement* avim_right_{nullptr};
  GstElement* rec_sink_right_{nullptr};

  GstBus* bus_{nullptr};
  guint bus_watch_id_{0};
  GMainContext* context_{nullptr};
  GMainLoop* loop_{nullptr};
  std::thread loop_thread_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoStreamerGstNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
