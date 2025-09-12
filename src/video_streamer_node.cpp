#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gst/gst.h>
#include <glib.h>

#include <string>
#include <sstream>
#include <thread>
#include <memory>
#include <mutex>
#include <filesystem>

class VideoStreamerNode : public rclcpp::Node {
public:
    VideoStreamerNode() : rclcpp::Node("video_streamer") {
        // Parameters
        this->declare_parameter<std::string>("cam_a", "/dev/v4l/by-id/unknown-cam-a");
        this->declare_parameter<std::string>("cam_b", "/dev/v4l/by-id/unknown-cam-b");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 15);
        this->declare_parameter<std::string>("raw_format", "UYVY");
        this->declare_parameter<std::string>("stream_host", "192.168.168.100");
        this->declare_parameter<int>("stream_port", 5600);
        this->declare_parameter<int>("bitrate_a_kbps", 400);
        this->declare_parameter<int>("bitrate_b_kbps", 300);
        this->declare_parameter<std::string>("out_dir", "/home/roofus/videos");
        this->declare_parameter<int>("segment_seconds", 60);
        this->declare_parameter<bool>("start_recording", false);
        this->declare_parameter<bool>("use_vaapi", true);
        this->declare_parameter<int>("rtp_mtu", 1200);

        cam_a_            = this->get_parameter("cam_a").as_string();
        cam_b_            = this->get_parameter("cam_b").as_string();
        width_            = this->get_parameter("width").as_int();
        height_           = this->get_parameter("height").as_int();
        fps_              = this->get_parameter("fps").as_int();
        stream_host_      = this->get_parameter("stream_host").as_string();
        stream_port_      = this->get_parameter("stream_port").as_int();
        bitrate_a_kbps_   = this->get_parameter("bitrate_a_kbps").as_int();
        bitrate_b_kbps_   = this->get_parameter("bitrate_b_kbps").as_int();
        out_dir_          = this->get_parameter("out_dir").as_string();
        segment_seconds_  = this->get_parameter("segment_seconds").as_int();
        recording_a_ = recording_b_ = this->get_parameter("start_recording").as_bool();
        use_vaapi_        = this->get_parameter("use_vaapi").as_bool();
        rtp_mtu_          = this->get_parameter("rtp_mtu").as_int();
        raw_format_       = this->get_parameter("raw_format").as_string();

        // Ensure output directory exists
        try {
            std::filesystem::create_directories(out_dir_);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to create out_dir '%s': %s", out_dir_.c_str(), e.what());
        }

        // Init GStreamer
        gst_init(nullptr, nullptr);

        // Build and start pipelines
        build_pipelines();
        start_pipelines();

        // Services
        srv_toggle_ = this->create_service<std_srvs::srv::Trigger>(
            "video_record_toggle",
            std::bind(&VideoStreamerNode::on_toggle, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_both_ = this->create_service<std_srvs::srv::SetBool>(
            "video_record_set",
            std::bind(&VideoStreamerNode::on_set_both, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_a_ = this->create_service<std_srvs::srv::SetBool>(
            "video_record_set_cam_a",
            std::bind(&VideoStreamerNode::on_set_a, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_b_ = this->create_service<std_srvs::srv::SetBool>(
            "video_record_set_cam_b",
            std::bind(&VideoStreamerNode::on_set_b, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "video_streamer started: stream %s:%d, VAAPI=%d", stream_host_.c_str(), stream_port_, use_vaapi_);
    }

    ~VideoStreamerNode() override {
        stop_pipelines();
    }

private:
    // ============ Pipeline construction ============
    std::string build_cam_a_pipeline_string(bool start_rec) {
        const int kf_stream = std::max(2 * fps_, 2); // keyframe about 2x fps
        const char *fmt_before_enc = use_vaapi_ ? "NV12" : "I420";

        std::ostringstream enc;
        if (use_vaapi_) {
            enc << "vaapih264enc tune=low-power rate-control=cbr max-bframes=0 bitrate=" << bitrate_a_kbps_
                << " keyframe-period=" << kf_stream;
        } else {
            enc << "x264enc tune=zerolatency speed-preset=ultrafast bframes=0 bitrate=" << bitrate_a_kbps_
                << " key-int-max=" << kf_stream;
        }

        std::ostringstream ss;
        ss
            << "v4l2src device=" << cam_a_
            << " ! video/x-raw,format=" << raw_format_ << ",width=" << width_ << ",height=" << height_ << ",framerate=" << fps_ << "/1"
            << " ! videoconvert ! video/x-raw,format=" << fmt_before_enc
            << " ! queue"
            << " ! " << enc.str()
            << " ! tee name=t_a "
            // Stream branch
            << "t_a. ! queue ! rtph264pay pt=96 mtu=" << rtp_mtu_ << " config-interval=1 ! udpsink host=" << stream_host_ << " port=" << stream_port_ << " sync=false"
            // Record branch
            << " t_a. ! queue leaky=downstream max-size-time=2000000000 ! valve name=valve_a drop=" << (start_rec ? "false" : "true")
            << " ! h264parse ! splitmuxsink location=" << out_dir_ << "/camA-%02d.mkv max-size-time=" << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
            << " muxer-factory=matroskamux";

        return ss.str();
    }

    std::string build_cam_b_pipeline_string(bool start_rec) {
        const int kf_record = std::max(2 * fps_, 2);
        const char *fmt_before_enc = use_vaapi_ ? "NV12" : "I420";

        std::ostringstream enc;
        if (use_vaapi_) {
            enc << "vaapih264enc tune=low-power rate-control=cbr max-bframes=2 bitrate=" << bitrate_b_kbps_
                << " keyframe-period=" << kf_record;
        } else {
            enc << "x264enc speed-preset=veryfast bframes=2 bitrate=" << bitrate_b_kbps_
                << " key-int-max=" << kf_record;
        }

        std::ostringstream ss;
        ss
            << "v4l2src device=" << cam_b_
            << " ! video/x-raw,format=" << raw_format_ << ",width=" << width_ << ",height=" << height_ << ",framerate=" << fps_ << "/1"
            << " ! videoconvert ! video/x-raw,format=" << fmt_before_enc
            << " ! queue"
            << " ! " << enc.str()
            << " ! queue leaky=downstream max-size-time=2000000000"
            << " ! valve name=valve_b drop=" << (start_rec ? "false" : "true")
            << " ! h264parse ! splitmuxsink location=" << out_dir_ << "/camB-%02d.mkv max-size-time=" << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
            << " muxer-factory=matroskamux";

        return ss.str();
    }

    void build_pipelines() {
        std::string a_str = build_cam_a_pipeline_string(recording_a_);
        std::string b_str = build_cam_b_pipeline_string(recording_b_);

        GError *err = nullptr;
        pipeline_a_ = gst_parse_launch(a_str.c_str(), &err);
        if (!pipeline_a_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create Cam A pipeline: %s", err ? err->message : "unknown");
            if (err) g_error_free(err);
            throw std::runtime_error("Failed to create Cam A pipeline");
        }
        if (err) { RCLCPP_WARN(this->get_logger(), "Cam A pipeline warnings: %s", err->message); g_error_free(err); err = nullptr; }

        pipeline_b_ = gst_parse_launch(b_str.c_str(), &err);
        if (!pipeline_b_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create Cam B pipeline: %s", err ? err->message : "unknown");
            if (err) g_error_free(err);
            throw std::runtime_error("Failed to create Cam B pipeline");
        }
        if (err) { RCLCPP_WARN(this->get_logger(), "Cam B pipeline warnings: %s", err->message); g_error_free(err); err = nullptr; }

        // Cache valve elements for fast toggling
        valve_a_ = gst_bin_get_by_name(GST_BIN(pipeline_a_), "valve_a");
        valve_b_ = gst_bin_get_by_name(GST_BIN(pipeline_b_), "valve_b");
        if (!valve_a_ || !valve_b_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to find valves in pipelines");
            throw std::runtime_error("Valve elements not found");
        }

        // Setup bus watches with dedicated GLib loop
        context_ = g_main_context_new();
        loop_ = g_main_loop_new(context_, FALSE);

        GstBus *bus_a = gst_element_get_bus(pipeline_a_);
        GstBus *bus_b = gst_element_get_bus(pipeline_b_);

        gst_bus_add_watch_full(bus_a, G_PRIORITY_DEFAULT, &VideoStreamerNode::bus_callback_static, this, nullptr);
        gst_bus_add_watch_full(bus_b, G_PRIORITY_DEFAULT, &VideoStreamerNode::bus_callback_static, this, nullptr);

        gst_object_unref(bus_a);
        gst_object_unref(bus_b);
    }

    void start_pipelines() {
        // Start GLib main loop thread first
        loop_thread_ = std::thread([this]() {
            g_main_context_push_thread_default(context_);
            g_main_loop_run(loop_);
            g_main_context_pop_thread_default(context_);
        });

        if (gst_element_set_state(pipeline_a_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Cam A pipeline to PLAYING");
            throw std::runtime_error("Cam A failed to PLAYING");
        }
        if (gst_element_set_state(pipeline_b_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Cam B pipeline to PLAYING");
            throw std::runtime_error("Cam B failed to PLAYING");
        }

        RCLCPP_INFO(this->get_logger(), "Pipelines PLAYING. Recording A=%d, B=%d", recording_a_, recording_b_);
    }

    void stop_pipelines() {
        // Stop pipelines and loop safely
        if (pipeline_a_) {
            gst_element_set_state(pipeline_a_, GST_STATE_NULL);
        }
        if (pipeline_b_) {
            gst_element_set_state(pipeline_b_, GST_STATE_NULL);
        }

        if (loop_) {
            g_main_loop_quit(loop_);
        }
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }

        if (valve_a_) { gst_object_unref(valve_a_); valve_a_ = nullptr; }
        if (valve_b_) { gst_object_unref(valve_b_); valve_b_ = nullptr; }
        if (pipeline_a_) { gst_object_unref(pipeline_a_); pipeline_a_ = nullptr; }
        if (pipeline_b_) { gst_object_unref(pipeline_b_); pipeline_b_ = nullptr; }
        if (loop_) { g_main_loop_unref(loop_); loop_ = nullptr; }
        if (context_) { g_main_context_unref(context_); context_ = nullptr; }
    }

    // ============ Services ============
    void on_toggle(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const bool any_on = recording_a_ || recording_b_;
        set_recording_both(!any_on);
        res->success = true;
        std::ostringstream msg;
        msg << "Recording " << (recording_a_ ? "A:on" : "A:off") << ", " << (recording_b_ ? "B:on" : "B:off");
        res->message = msg.str();
    }

    void on_set_both(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        set_recording_both(req->data);
        res->success = true;
        res->message = std::string("Recording both ") + (req->data ? "ON" : "OFF");
    }

    void on_set_a(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        set_recording_single(true, req->data);
        res->success = true;
        res->message = std::string("Recording A ") + (req->data ? "ON" : "OFF");
    }

    void on_set_b(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        set_recording_single(false, req->data);
        res->success = true;
        res->message = std::string("Recording B ") + (req->data ? "ON" : "OFF");
    }

    void set_recording_both(bool enable) {
        set_recording_single(true, enable);
        set_recording_single(false, enable);
    }

    void set_recording_single(bool is_a, bool enable) {
        std::lock_guard<std::mutex> lock(valve_mutex_);
        GstElement *valve = is_a ? valve_a_ : valve_b_;
        if (!valve) return;

        // drop=false => recording ON (let data through)
        gboolean drop = enable ? FALSE : TRUE;
        g_object_set(G_OBJECT(valve), "drop", drop, NULL);
        if (is_a) {
            recording_a_ = enable;
        } else {
            recording_b_ = enable;
        }
        RCLCPP_INFO(this->get_logger(), "%s recording %s", is_a ? "Cam A" : "Cam B", enable ? "ON" : "OFF");
    }

    // ============ GStreamer bus logging ============
    static gboolean bus_callback_static(GstBus *bus, GstMessage *msg, gpointer user_data) {
        (void)bus;
        return static_cast<VideoStreamerNode*>(user_data)->bus_callback(msg);
    }

    gboolean bus_callback(GstMessage *msg) {
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = nullptr; gchar *dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                RCLCPP_ERROR(this->get_logger(), "GStreamer ERROR from %s: %s", GST_OBJECT_NAME(msg->src), err ? err->message : "unknown");
                if (dbg) { RCLCPP_ERROR(this->get_logger(), "Debug: %s", dbg); g_free(dbg); }
                if (err) g_error_free(err);
                break;
            }
            case GST_MESSAGE_EOS:
                RCLCPP_WARN(this->get_logger(), "Pipeline EOS from %s", GST_OBJECT_NAME(msg->src));
                break;
            default:
                break;
        }
        return TRUE; // keep watching
    }

    // Params
    std::string cam_a_;
    std::string cam_b_;
    int width_{};
    int height_{};
    int fps_{};
    std::string stream_host_;
    int stream_port_{};
    int bitrate_a_kbps_{};
    int bitrate_b_kbps_{};
    std::string out_dir_;
    int segment_seconds_{};
    bool use_vaapi_{};
    int rtp_mtu_{};

    // State
    bool recording_a_{false};
    bool recording_b_{false};

    // GStreamer
    GstElement *pipeline_a_{nullptr};
    GstElement *pipeline_b_{nullptr};
    GstElement *valve_a_{nullptr};
    GstElement *valve_b_{nullptr};
    GMainContext *context_{nullptr};
    GMainLoop *loop_{nullptr};
    std::thread loop_thread_;
    std::mutex valve_mutex_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_toggle_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_both_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_a_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_b_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoStreamerNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}


