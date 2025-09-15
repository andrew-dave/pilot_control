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
        this->declare_parameter<int>("record_bitrate_kbps", 10000);

        cam_a_            = this->get_parameter("cam_a").as_string();
        cam_b_            = this->get_parameter("cam_b").as_string();
        out_dir_          = this->get_parameter("out_dir").as_string();
        segment_seconds_  = this->get_parameter("segment_seconds").as_int();
        recording_a_ = recording_b_ = this->get_parameter("start_recording").as_bool();
        record_w_ = this->get_parameter("record_width").as_int();
        record_h_ = this->get_parameter("record_height").as_int();
        record_fps_ = this->get_parameter("record_fps").as_int();
        record_bitrate_kbps_ = this->get_parameter("record_bitrate_kbps").as_int();

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

        // Service - only keep the record control for both cameras
        srv_record_control_ = this->create_service<std_srvs::srv::SetBool>(
            "video_record_set",
            std::bind(&VideoStreamerNode::on_record_control, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "video_recorder started: re-encode H.264, %dx%d@%d, ~%d kbps to %s",
                    record_w_, record_h_, record_fps_, record_bitrate_kbps_, out_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Service available: /video_record_set (controls recording for both cameras)");
    }

    ~VideoStreamerNode() override {
        stop_pipelines();
    }

private:
    // ============ Pipeline construction ============
    std::string build_cam_a_pipeline_string(bool start_rec) {
        // Record pipeline: decode MJPEG -> rate limit -> H.264 encode -> MKV
        const int keyint = std::max(2 * record_fps_, 2);
        std::ostringstream ss;
        ss << "v4l2src device=" << cam_a_ << " do-timestamp=true "
           << "! image/jpeg,width=" << record_w_ << ",height=" << record_h_ << " "
           << "! jpegparse ! jpegdec "
           << "! videorate ! video/x-raw,framerate=" << record_fps_ << "/1 "
           << "! queue max-size-buffers=0 max-size-bytes=0 max-size-time=4000000000 "
           << "! valve name=valve_a drop=" << (start_rec ? "false" : "true") << " "
           << "! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << record_bitrate_kbps_
           << " key-int-max=" << keyint << " "
           << "! h264parse ! splitmuxsink location=" << out_dir_ << "/camA-%02d.mkv max-size-time="
           << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
           << " muxer-factory=matroskamux";
        return ss.str();
    }

    std::string build_cam_b_pipeline_string(bool start_rec) {
        // Record pipeline: decode MJPEG -> rate limit -> H.264 encode -> MKV
        const int keyint = std::max(2 * record_fps_, 2);
        std::ostringstream ss;
        ss << "v4l2src device=" << cam_b_ << " do-timestamp=true "
           << "! image/jpeg,width=" << record_w_ << ",height=" << record_h_ << " "
           << "! jpegparse ! jpegdec "
           << "! videorate ! video/x-raw,framerate=" << record_fps_ << "/1 "
           << "! queue max-size-buffers=0 max-size-bytes=0 max-size-time=4000000000 "
           << "! valve name=valve_b drop=" << (start_rec ? "false" : "true") << " "
           << "! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << record_bitrate_kbps_
           << " key-int-max=" << keyint << " "
           << "! h264parse ! splitmuxsink location=" << out_dir_ << "/camB-%02d.mkv max-size-time="
           << static_cast<unsigned long long>(segment_seconds_) * 1000000000ULL
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
    int record_bitrate_kbps_{};

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


