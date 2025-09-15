#include <rclcpp/rclcpp.hpp>
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
        this->declare_parameter<std::string>("out_dir", "/home/roofus/videos");
        this->declare_parameter<int>("segment_seconds", 60);
        this->declare_parameter<bool>("start_recording", false);
        // Recording-only defaults target 1920x1200@60 (per v4l2 capabilities provided)
        // Record resolution (use highest by default)
        this->declare_parameter<int>("record_width", 1920);
        this->declare_parameter<int>("record_height", 1200);
        this->declare_parameter<int>("record_fps", 30);

        cam_a_            = this->get_parameter("cam_a").as_string();
        cam_b_            = this->get_parameter("cam_b").as_string();
        out_dir_          = this->get_parameter("out_dir").as_string();
        segment_seconds_  = this->get_parameter("segment_seconds").as_int();
        recording_a_ = recording_b_ = this->get_parameter("start_recording").as_bool();
        record_w_ = this->get_parameter("record_width").as_int();
        record_h_ = this->get_parameter("record_height").as_int();
        record_fps_ = this->get_parameter("record_fps").as_int();

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
        // Force both pipelines to use the same system clock for consistent timestamping
        {
            GstClock *sysclk = gst_system_clock_obtain();
            if (sysclk) {
                if (pipeline_a_) gst_pipeline_use_clock(GST_PIPELINE(pipeline_a_), sysclk);
                if (pipeline_b_) gst_pipeline_use_clock(GST_PIPELINE(pipeline_b_), sysclk);
                gst_object_unref(sysclk);
            }
        }
        start_pipelines();

        // Service - only keep the record control for both cameras
        srv_record_control_ = this->create_service<std_srvs::srv::SetBool>(
            "video_record_set",
            std::bind(&VideoStreamerNode::on_record_control, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "video_recorder started: MJPEG direct, %dx%d@%d to %s",
                    record_w_, record_h_, record_fps_, out_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Service available: /video_record_set (controls recording for both cameras)");
    }

    ~VideoStreamerNode() override {
        stop_pipelines();
    }

private:
    // ============ Helpers ============
    static std::string now_timestamp_string() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%04d%02d%02d-%02d%02d%02d",
                      tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                      tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
        return std::string(buf);
    }

    // ============ Pipeline construction ============
    std::string build_cam_a_pipeline_string(bool start_rec) {
        // Simplest record pipeline: MJPEG direct -> MKV
        std::ostringstream ss;
        ss << "v4l2src device=" << cam_a_ << " do-timestamp=true "
           << "! image/jpeg,width=" << record_w_ << ",height=" << record_h_ << ",framerate=" << record_fps_ << "/1 "
           << "! jpegparse "
           << "! valve name=valve_a drop=" << (start_rec ? "false" : "true") << " "
           << "! queue "
           << "! splitmuxsink name=splitmux_a location=" << out_dir_ << "/" << file_base_a_ << "-%02d.mkv max-size-time="
           << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
           << " muxer-factory=matroskamux";
        return ss.str();
    }

    std::string build_cam_b_pipeline_string(bool start_rec) {
        // Simplest record pipeline: MJPEG direct -> MKV
        std::ostringstream ss;
        ss << "v4l2src device=" << cam_b_ << " do-timestamp=true "
           << "! image/jpeg,width=" << record_w_ << ",height=" << record_h_ << ",framerate=" << record_fps_ << "/1 "
           << "! jpegparse "
           << "! valve name=valve_b drop=" << (start_rec ? "false" : "true") << " "
           << "! queue "
           << "! splitmuxsink name=splitmux_b location=" << out_dir_ << "/" << file_base_b_ << "-%02d.mkv max-size-time="
           << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
           << " muxer-factory=matroskamux";
        return ss.str();
    }

    void build_pipelines() {
        // Build with unique timestamped base names
        file_base_a_ = std::string("camA-") + now_timestamp_string();
        file_base_b_ = std::string("camB-") + now_timestamp_string();
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

        // Cache valve and splitmux elements for fast toggling and splitting
        valve_a_ = gst_bin_get_by_name(GST_BIN(pipeline_a_), "valve_a");
        valve_b_ = gst_bin_get_by_name(GST_BIN(pipeline_b_), "valve_b");
        splitmux_a_ = gst_bin_get_by_name(GST_BIN(pipeline_a_), "splitmux_a");
        splitmux_b_ = gst_bin_get_by_name(GST_BIN(pipeline_b_), "splitmux_b");
        if (!valve_a_ || !valve_b_ || !splitmux_a_ || !splitmux_b_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to find required elements (valves or splitmux) in pipelines");
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
        if (splitmux_a_) { gst_object_unref(splitmux_a_); splitmux_a_ = nullptr; }
        if (splitmux_b_) { gst_object_unref(splitmux_b_); splitmux_b_ = nullptr; }
        if (pipeline_a_) { gst_object_unref(pipeline_a_); pipeline_a_ = nullptr; }
        if (pipeline_b_) { gst_object_unref(pipeline_b_); pipeline_b_ = nullptr; }
        if (loop_) { g_main_loop_unref(loop_); loop_ = nullptr; }
        if (context_) { g_main_context_unref(context_); context_ = nullptr; }
    }

    // ============ Service ============
    void on_record_control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        set_recording_both(req->data);
        res->success = true;
        res->message = std::string("Recording both cameras ") + (req->data ? "ON" : "OFF");
    }

    // Toggle both valves within the GLib main context to synchronize start/stop
    void set_recording_both(bool enable) {
        struct ToggleData { VideoStreamerNode* self; gboolean drop; };
        auto *data = new ToggleData{ this, enable ? FALSE : TRUE };
        g_main_context_invoke(context_, [](gpointer user_data) -> gboolean {
            auto *d = static_cast<ToggleData*>(user_data);
            VideoStreamerNode *self = d->self;
            {
                std::lock_guard<std::mutex> lock(self->valve_mutex_);
                if (self->valve_a_) g_object_set(G_OBJECT(self->valve_a_), "drop", d->drop, NULL);
                if (self->valve_b_) g_object_set(G_OBJECT(self->valve_b_), "drop", d->drop, NULL);
                const bool enable_local = (d->drop == FALSE);
                self->recording_a_ = enable_local;
                self->recording_b_ = enable_local;
            }
            // If stopping, request split and wait for it; if starting, also split to force a fresh file
            if (self->splitmux_a_) g_signal_emit_by_name(self->splitmux_a_, "split-now");
            if (self->splitmux_b_) g_signal_emit_by_name(self->splitmux_b_, "split-now");
            delete d;
            return G_SOURCE_REMOVE;
        }, data);
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
    std::string out_dir_;
    int segment_seconds_{};
    // Record pipeline controls
    int record_w_{};
    int record_h_{};
    int record_fps_{};
    std::string file_base_a_;
    std::string file_base_b_;

    // State
    bool recording_a_{false};
    bool recording_b_{false};
    

    // GStreamer
    GstElement *pipeline_a_{nullptr};
    GstElement *pipeline_b_{nullptr};
    GstElement *valve_a_{nullptr};
    GstElement *valve_b_{nullptr};
    GstElement *splitmux_a_{nullptr};
    GstElement *splitmux_b_{nullptr};
    GMainContext *context_{nullptr};
    GMainLoop *loop_{nullptr};
    std::thread loop_thread_;
    std::mutex valve_mutex_;

    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_record_control_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoStreamerNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}


