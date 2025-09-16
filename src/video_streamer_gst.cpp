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

class VideoStreamerGstNode : public rclcpp::Node {
public:
  VideoStreamerGstNode() : rclcpp::Node("video_streamer_gst"), recording_active_(false), shutting_down_(false) {
    // Parameters
    this->declare_parameter<std::string>("left_device", "/dev/v4l/by-id/See3CAM_Left-video-index0");
    this->declare_parameter<std::string>("right_device", "/dev/v4l/by-id/See3CAM_Right-video-index0");
    this->declare_parameter<std::string>("camera_label", "cam_left");
    this->declare_parameter<std::string>("output_dir", default_output_dir());
    this->declare_parameter<std::string>("fourcc", "MJPG");
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<bool>("start_recording", false);

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
    record_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/video_record_set",
        std::bind(&VideoStreamerGstNode::onSetRecording, this, std::placeholders::_1, std::placeholders::_2));

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

    const double rec_fps_d = this->get_parameter("fps").as_double();
    const int rec_fps = static_cast<int>(rec_fps_d);

    const std::string stream_host = this->get_parameter("stream_host").as_string();
    const int stream_port = this->get_parameter("stream_port").as_int();
    const int stream_bitrate = this->get_parameter("stream_bitrate_kbps").as_int();
    const int rtp_mtu = this->get_parameter("rtp_mtu").as_int();

    std::ostringstream oss;
    oss << (use_mjpeg
              ? "v4l2src device=" + device +
                    " ! image/jpeg,width=" + std::to_string(cap_w) + ",height=" + std::to_string(cap_h) + ",framerate=" + std::to_string(cap_fps) + "/1 "
                    "! jpegdec ! videoconvert ! video/x-raw,format=I420 "
              : "v4l2src device=" + device +
                    " ! video/x-raw,format=" + raw_format + ",width=" + std::to_string(raw_w) + ",height=" + std::to_string(raw_h) + ",framerate=" + std::to_string(raw_fps) + "/1 "
                    "! videoconvert ! video/x-raw,format=I420 ")
        << "! tee name=T ";

    // Record branch (non-leaky)
    oss << " T. ! queue ! videorate ! video/x-raw,framerate=" << rec_fps << "/1 "
        << "! videoconvert ! video/x-raw,format=I420 "
        << "! valve name=valve_rec drop=true "
        << "! jpegenc quality=95 ! avimux name=avim ! filesink name=rec_sink async=false sync=false ";

    // Stream branch (leaky, exact chain requested)
    oss << " T. ! queue leaky=downstream max-size-buffers=120 max-size-bytes=0 max-size-time=0 "
        << "! videorate ! video/x-raw,framerate=15/1 "
        << "! videoscale ! video/x-raw,width=640,height=480 "
        << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << stream_bitrate << " key-int-max=30 bframes=0 "
        << "! video/x-h264,stream-format=byte-stream,alignment=au "
        << "! rtph264pay pt=96 config-interval=1 mtu=" << rtp_mtu << " "
        << "! udpsink host=" << stream_host << " port=" << stream_port << " sync=false";

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

    pipeline_.reset(pipe, [](GstElement *p) { if (p) gst_object_unref(p); });

    // Grab elements we control
    valve_rec_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "valve_rec"));
    avim_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "avim"));
    rec_sink_ = GST_ELEMENT(gst_bin_get_by_name(GST_BIN(pipeline_.get()), "rec_sink"));

    if (!valve_rec_ || !avim_ || !rec_sink_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to retrieve required elements (valve_rec/avim/rec_sink)");
      throw std::runtime_error("Missing elements");
    }

    // Set pipeline to PLAYING
    GstStateChangeReturn ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set pipeline to PLAYING");
      throw std::runtime_error("Pipeline PLAYING failed");
    }
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

    GSource *bus_source = gst_bus_create_watch(bus_);
    g_source_set_callback(bus_source, (GSourceFunc)&VideoStreamerGstNode::bus_source_func, this, nullptr);
    g_source_attach(bus_source, context_);
    g_source_unref(bus_source);

    loop_thread_ = std::thread([this]() {
      g_main_context_push_thread_default(context_);
      g_main_loop_run(loop_);
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

  std::string build_output_path() const {
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

  bool start_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;

    const std::string location = build_output_path();
    g_object_set(G_OBJECT(rec_sink_), "location", location.c_str(), NULL);
    RCLCPP_INFO(this->get_logger(), "Recording to: %s", location.c_str());

    // Ensure recording branch is in PLAYING (in case it was readied after a previous stop)
    gst_element_set_state(avim_, GST_STATE_PLAYING);
    gst_element_set_state(rec_sink_, GST_STATE_PLAYING);

    // Open valve
    g_object_set(G_OBJECT(valve_rec_), "drop", FALSE, NULL);
    recording_active_ = true;
    return true;
  }

  bool stop_recording() {
    if (!pipeline_ || !valve_rec_ || !avim_ || !rec_sink_) return false;
    if (!recording_active_) return true;

    // Close valve to stop new buffers
    g_object_set(G_OBJECT(valve_rec_), "drop", TRUE, NULL);

    // Send EOS to finalize AVI cleanly
    eos_from_avim_received_ = false;
    if (!gst_element_send_event(avim_, gst_event_new_eos())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send EOS to avim branch");
    }

    // Wait for EOS confirmation for a bounded time
    {
      std::unique_lock<std::mutex> ul(cv_mu_);
      cv_.wait_for(ul, std::chrono::seconds(3), [this]() { return eos_from_avim_received_.load(); });
    }

    // Ready branch for next session
    gst_element_set_state(rec_sink_, GST_STATE_READY);
    gst_element_set_state(avim_, GST_STATE_READY);
    // Set back to PLAYING so next start_recording just opens the valve
    gst_element_set_state(avim_, GST_STATE_PLAYING);
    gst_element_set_state(rec_sink_, GST_STATE_PLAYING);

    recording_active_ = false;
    return true;
  }

  // ===== Bus watch =====
  static gboolean bus_source_func(GstBus * /*bus*/, GstMessage *msg, gpointer user_data) {
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
        // Identify EOS from avim branch
        GstObject *src = GST_MESSAGE_SRC(msg);
        if (src && self->avim_ && (src == GST_OBJECT(self->avim_) || gst_object_has_ancestor(src, GST_OBJECT(self->avim_)))) {
          std::lock_guard<std::mutex> lk(self->cv_mu_);
          self->eos_from_avim_received_.store(true);
          self->cv_.notify_all();
        } else {
          // Pipeline-wide EOS (should not happen in normal run)
          RCLCPP_WARN(self->get_logger(), "EOS received (non-recording branch)");
        }
        break;
      }
      default:
        break;
    }
    return TRUE;
  }

  // ===== Members =====
  std::mutex mu_;
  std::atomic<bool> recording_active_;
  std::atomic<bool> shutting_down_;

  // EOS sync for recording branch
  std::mutex cv_mu_;
  std::condition_variable cv_;
  std::atomic<bool> eos_from_avim_received_{false};

  // GStreamer bits
  struct GstElementDeleter { void operator()(GstElement *p) const { if (p) gst_object_unref(p); } };
  std::unique_ptr<GstElement, GstElementDeleter> pipeline_;
  GstElement *valve_rec_ {nullptr};
  GstElement *avim_ {nullptr};
  GstElement *rec_sink_ {nullptr};
  GstBus *bus_ {nullptr};
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


