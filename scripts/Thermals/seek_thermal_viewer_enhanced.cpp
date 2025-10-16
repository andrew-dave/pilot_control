// seek_thermal_viewer_enhanced.cpp
//
// Build (Ubuntu):
//   g++ -std=c++17 -O2 seek_thermal_viewer_enhanced.cpp -o seek_thermal_viewer_enhanced
//       $(pkg-config --cflags --libs opencv4) -L/usr/lib -Wl,-rpath,/usr/lib -lseekcamera

#include "seekcamera.h"
#include "seekcamera_manager.h"
#include "seekcamera_frame.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ===================== SDK compatibility shims =====================
// Map the actual SDK enum names to our compatibility names
#define SEEKCAMERA_PIPELINE_MODE_IMAGE_LITE      SEEKCAMERA_IMAGE_LITE
#define SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY    SEEKCAMERA_IMAGE_LEGACY  
#define SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION SEEKCAMERA_IMAGE_SEEKVISION

// ======================= Configuration =========================
struct ThermalConfig {
    int color_palette = 0;   // 0=JET, 1=HOT, 2=COOL, 3=RAINBOW, 4=VIRIDIS, 5=INFERNO
    int pipeline_mode = 2;   // 0=IMAGE_LITE, 1=IMAGE_LEGACY, 2=IMAGE_SEEKVISION
    int agc_mode = 0;        // 0=LINEAR, 1=HISTEQ
    int target_fps = 30;     // Display throttling
    bool show_temperature = true;
    bool show_fps = true;
    bool show_center_temp = true;
    bool show_pipeline_info = true;
    float histeq_plateau = 0.10f; // HistEq only
};

// ======================= Globals ===============================
static ThermalConfig config;
static std::string window_name = "Seek Thermal Camera - Enhanced";

static seekcamera_t* camera_handle = nullptr;

static std::mutex thermal_mutex;
static cv::Mat    thermal32f;                 // Latest radiometric frame (CV_32F)
static std::atomic<bool> thermal_ready{false};

static std::atomic<bool> should_exit{false};
static std::atomic<bool> camera_connected{false};

static std::atomic<uint64_t> frames_total{0}; // processed frames (from callback)
static std::atomic<uint64_t> display_count{0};

static std::atomic<uint64_t> fps_counter{0};  // per-second FPS window
static std::chrono::steady_clock::time_point start_time;
static std::chrono::steady_clock::time_point fps_start_time;

// OpenCV colormaps and names
static const std::vector<int> colormaps = {
    cv::COLORMAP_JET,     // 0
    cv::COLORMAP_HOT,     // 1
    cv::COLORMAP_COOL,    // 2
    cv::COLORMAP_RAINBOW, // 3
    cv::COLORMAP_VIRIDIS, // 4
    cv::COLORMAP_INFERNO  // 5
};

static const std::vector<std::string> colormap_names = {
    "JET", "HOT", "COOL", "RAINBOW", "VIRIDIS", "INFERNO"
};

// Pretty names for pipeline/AGC
static const std::vector<std::string> pipeline_names = {
    "IMAGE_LITE", "IMAGE_LEGACY", "IMAGE_SEEKVISION"
};
static const std::vector<std::string> agc_names = {
    "LINEAR", "HISTEQ"
};

// Mapping from our indices to the SDK enums (works with both naming schemes)
static const seekcamera_pipeline_mode_t PIPELINE_TABLE[3] = {
    SEEKCAMERA_PIPELINE_MODE_IMAGE_LITE,
    SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY,
    SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION
};
static const seekcamera_agc_mode_t AGC_TABLE[2] = {
    SEEKCAMERA_AGC_MODE_LINEAR,
    SEEKCAMERA_AGC_MODE_HISTEQ
};

static std::string pipeline_enum_to_string(seekcamera_pipeline_mode_t m) {
    switch (m) {
        case SEEKCAMERA_PIPELINE_MODE_IMAGE_LITE:      return "IMAGE_LITE";
        case SEEKCAMERA_PIPELINE_MODE_IMAGE_LEGACY:    return "IMAGE_LEGACY";
        case SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION:return "IMAGE_SEEKVISION";
        default: return "UNKNOWN";
    }
}
static std::string agc_enum_to_string(seekcamera_agc_mode_t a) {
    switch (a) {
        case SEEKCAMERA_AGC_MODE_LINEAR: return "LINEAR";
        case SEEKCAMERA_AGC_MODE_HISTEQ: return "HISTEQ";
        default: return "UNKNOWN";
    }
}

// ================== Camera Settings Helpers ====================
static void apply_camera_settings() {
    if (!camera_handle) return;

    // Pipeline mode
    seekcamera_pipeline_mode_t pmode = PIPELINE_TABLE[std::clamp(config.pipeline_mode, 0, 2)];
    seekcamera_error_t err = seekcamera_set_pipeline_mode(camera_handle, pmode);
    if (err == SEEKCAMERA_SUCCESS) {
        std::cout << "Pipeline mode set to: " << pipeline_names[config.pipeline_mode] << "\n";
    } else {
        std::cout << "Failed to set pipeline mode (err=" << err << ")\n";
    }

    // AGC (not adjustable under SeekVision)
    if (pmode != SEEKCAMERA_PIPELINE_MODE_IMAGE_SEEKVISION) {
        seekcamera_agc_mode_t agc = AGC_TABLE[std::clamp(config.agc_mode, 0, 1)];
        err = seekcamera_set_agc_mode(camera_handle, agc);
        if (err == SEEKCAMERA_SUCCESS) {
            std::cout << "AGC mode set to: " << agc_names[config.agc_mode] << "\n";
        } else {
            std::cout << "Failed to set AGC mode (err=" << err << ")\n";
        }

        if (agc == SEEKCAMERA_AGC_MODE_HISTEQ) {
            err = seekcamera_set_histeq_agc_plateau(camera_handle, config.histeq_plateau);
            if (err == SEEKCAMERA_SUCCESS) {
                std::cout << "HistEq plateau set to: " << config.histeq_plateau << "\n";
            } else {
                std::cout << "Failed to set HistEq plateau (err=" << err << ")\n";
            }
        }
    } else {
        std::cout << "AGC mode: controlled by IMAGE_SEEKVISION (AGC setters ignored)\n";
    }
}

static void get_camera_settings() {
    if (!camera_handle) return;

    seekcamera_pipeline_mode_t pm;
    if (seekcamera_get_pipeline_mode(camera_handle, &pm) == SEEKCAMERA_SUCCESS) {
        std::cout << "Current pipeline mode: " << pipeline_enum_to_string(pm) << "\n";
    }
    seekcamera_agc_mode_t am;
    if (seekcamera_get_agc_mode(camera_handle, &am) == SEEKCAMERA_SUCCESS) {
        std::cout << "Current AGC mode: " << agc_enum_to_string(am) << "\n";
    }
    // HistEq plateau (only meaningful if HistEq)
    float plat = 0.f;
    if (seekcamera_get_histeq_agc_plateau(camera_handle, &plat) == SEEKCAMERA_SUCCESS) {
        std::cout << "Current HistEq plateau: " << plat << "\n";
    }
}

// ==================== SDK Callbacks ============================
static void on_frame(seekcamera_t* cam, seekcamera_frame_t* cam_frame, void* user) {
    (void)cam; (void)user;

    // LOCK: copy pixels out fast; do not block here
    seekcamera_frame_lock(cam_frame);

    seekframe_t* therm = nullptr;
    if (seekcamera_frame_get_frame_by_format(
            cam_frame, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &therm) == SEEKCAMERA_SUCCESS && therm) {

        const float* px = static_cast<const float*>(seekframe_get_data(therm));
        size_t w = seekframe_get_width(therm);
        size_t h = seekframe_get_height(therm);

        {
            std::lock_guard<std::mutex> lk(thermal_mutex);
            if (thermal32f.empty() ||
                thermal32f.cols != static_cast<int>(w) ||
                thermal32f.rows != static_cast<int>(h)) {
                thermal32f.create(static_cast<int>(h), static_cast<int>(w), CV_32F);
            }
            std::memcpy(thermal32f.data, px, w * h * sizeof(float));
        }

        thermal_ready = true;
        frames_total++;
    }

    seekcamera_frame_unlock(cam_frame);
}

static void on_event(seekcamera_t* cam, seekcamera_manager_event_t ev,
                     seekcamera_error_t status, void* user) {
    (void)status; (void)user;

    switch (ev) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT:
            std::cout << "Camera connected.\n";
            camera_connected = true;
            camera_handle = cam;

            // Register frame callback and request thermography float
            seekcamera_register_frame_available_callback(cam, on_frame, nullptr);
            {
                uint32_t fmt = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
                seekcamera_error_t err = seekcamera_capture_session_start(cam, fmt);
                if (err != SEEKCAMERA_SUCCESS) {
                    std::cerr << "Failed to start capture session (err=" << err << ")\n";
                }
            }

            get_camera_settings();
            apply_camera_settings();

            std::cout << "Controls:\n"
                      << "  ESC  - Exit\n"
                      << "  1..6 - Change color palette (JET/HOT/COOL/RAINBOW/VIRIDIS/INFERNO)\n"
                      << "  P    - Cycle pipeline mode (LITE/LEGACY/SEEKVISION)\n"
                      << "  A    - Toggle AGC mode (LINEAR/HISTEQ)\n"
                      << "  +/-  - Adjust HistEq plateau\n"
                      << "  T/F/C/I - Toggle overlays\n";
            break;

        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            // For Micro cores; harmless for Mosaic if never raised
            std::cout << "Camera ready to pair. Storing calibration...\n";
            seekcamera_store_calibration_data(cam, nullptr, nullptr, nullptr);
            break;

        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
            std::cout << "Camera disconnected.\n";
            camera_connected = false;
            if (camera_handle) {
                seekcamera_capture_session_stop(camera_handle);
            }
            camera_handle = nullptr;
            break;

        case SEEKCAMERA_MANAGER_EVENT_ERROR:
            std::cerr << "Camera manager error (code=" << status << ")\n";
            break;

        default:
            break;
    }
}

// ======================= UI / Usage ============================
static void print_usage() {
    std::cout << "Usage: ./seek_thermal_viewer_enhanced [options]\n"
              << "Options:\n"
              << "  -p <0-5>   Color palette   (0=JET,1=HOT,2=COOL,3=RAINBOW,4=VIRIDIS,5=INFERNO)\n"
              << "  -m <0-2>   Pipeline mode   (0=LITE,1=LEGACY,2=SEEKVISION)\n"
              << "  -a <0-1>   AGC mode        (0=LINEAR,1=HISTEQ)\n"
              << "  -f <fps>   Target display FPS (1..120)\n"
              << "  -t         Disable temperature overlay\n"
              << "  -s         Disable FPS overlay\n"
              << "  -c         Disable center temperature overlay\n"
              << "  -i         Disable pipeline/AGC overlay\n"
              << "  -h, --help Show this help\n";
}

// ========================= main() =============================
int main(int argc, char* argv[]) {
    // CLI parse
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-p" && i + 1 < argc) {
            config.color_palette = std::stoi(argv[++i]);
            if (config.color_palette < 0 || config.color_palette > 5) {
                std::cerr << "Invalid palette. Use 0..5.\n";
                return 1;
            }
        } else if (arg == "-m" && i + 1 < argc) {
            config.pipeline_mode = std::stoi(argv[++i]);
            if (config.pipeline_mode < 0 || config.pipeline_mode > 2) {
                std::cerr << "Invalid pipeline mode. Use 0..2.\n";
                return 1;
            }
        } else if (arg == "-a" && i + 1 < argc) {
            config.agc_mode = std::stoi(argv[++i]);
            if (config.agc_mode < 0 || config.agc_mode > 1) {
                std::cerr << "Invalid AGC mode. Use 0..1.\n";
                return 1;
            }
        } else if (arg == "-f" && i + 1 < argc) {
            config.target_fps = std::stoi(argv[++i]);
            if (config.target_fps < 1 || config.target_fps > 120) {
                std::cerr << "Invalid FPS. Use 1..120.\n";
                return 1;
            }
        } else if (arg == "-t") {
            config.show_temperature = false;
        } else if (arg == "-s") {
            config.show_fps = false;
        } else if (arg == "-c") {
            config.show_center_temp = false;
        } else if (arg == "-i") {
            config.show_pipeline_info = false;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage();
            return 1;
        }
    }

    std::cout << "Starting Enhanced Seek Thermal Camera Viewer...\n"
              << "  Color Palette : " << colormap_names[config.color_palette] << "\n"
              << "  Pipeline Mode : " << pipeline_names[config.pipeline_mode] << "\n"
              << "  AGC Mode      : "  << agc_names[config.agc_mode] << "\n"
              << "  Target FPS    : "  << config.target_fps << "\n";

    start_time = std::chrono::steady_clock::now();
    fps_start_time = start_time;

    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    // Create manager and register event callback
    seekcamera_manager_t* mgr = nullptr;
    if (seekcamera_manager_create(&mgr, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager.\n";
        return 1;
    }
    seekcamera_manager_register_event_callback(mgr, on_event, nullptr);
    std::cout << "Camera manager created. Waiting for camera...\n";

    // Main UI/render loop
    auto last_display_time = std::chrono::steady_clock::now();
    const auto display_interval = std::chrono::milliseconds(1000 / std::max(1, config.target_fps));

    while (!should_exit.load()) {
        auto now = std::chrono::steady_clock::now();

        // Render at target FPS
        if (now - last_display_time >= display_interval && thermal_ready.load()) {
            cv::Mat colored;
            double min_val = 0.0, max_val = 0.0, center_temp = 0.0;

            {
                std::lock_guard<std::mutex> lk(thermal_mutex);

                // Normalize & colorize for display
                cv::Mat u8;
                cv::normalize(thermal32f, u8, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::applyColorMap(u8, colored, colormaps[std::clamp(config.color_palette, 0, 5)]);

                if (config.show_temperature || config.show_center_temp) {
                    cv::minMaxLoc(thermal32f, &min_val, &max_val);
                    center_temp = thermal32f.at<float>(thermal32f.rows/2, thermal32f.cols/2);
                }
            }

            // Overlays (outside lock)
            int y = 25;
            if (config.show_temperature) {
                cv::putText(colored, "Min: " + cv::format("%.1fC", min_val),
                            {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2); y += 25;
                cv::putText(colored, "Max: " + cv::format("%.1fC", max_val),
                            {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2); y += 25;
            }
            if (config.show_center_temp) {
                cv::putText(colored, "Center: " + cv::format("%.1fC", center_temp),
                            {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2); y += 25;
            }
            if (config.show_fps) {
                fps_counter++;
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start_time).count();
                if (ms >= 1000) {
                    double fps = (fps_counter.load() * 1000.0) / ms;
                    cv::putText(colored, "FPS: " + cv::format("%.1f", fps),
                                {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 2);
                    y += 25;
                    fps_counter = 0;
                    fps_start_time = now;
                }
            }
            if (config.show_pipeline_info) {
                cv::putText(colored,
                            "Pipeline: " + pipeline_names[config.pipeline_mode] +
                            " | AGC: " + agc_names[config.agc_mode],
                            {10, colored.rows - 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200,200,200}, 1);
                cv::putText(colored,
                            "Colormap: " + colormap_names[config.color_palette],
                            {10, colored.rows - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200,200,200}, 1);
            }

            cv::imshow(window_name, colored);
            thermal_ready = false;
            last_display_time = now;
            display_count++;
        }

        // Keyboard
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) { // ESC
            std::cout << "ESC pressed. Exiting...\n";
            should_exit = true;
        } else if (key >= '1' && key <= '6') {
            config.color_palette = key - '1';
            std::cout << "Colormap: " << colormap_names[config.color_palette] << "\n";
        } else if (key == 'p' || key == 'P') {
            config.pipeline_mode = (config.pipeline_mode + 1) % 3;
            apply_camera_settings();
        } else if (key == 'a' || key == 'A') {
            config.agc_mode = (config.agc_mode + 1) % 2;
            apply_camera_settings();
        } else if (key == '+' || key == '=') {
            config.histeq_plateau = std::min(1.0f, config.histeq_plateau + 0.05f);
            if (config.agc_mode == 1 && config.pipeline_mode != 2) apply_camera_settings();
            std::cout << "HistEq plateau: " << std::fixed << std::setprecision(2) << config.histeq_plateau << "\n";
        } else if (key == '-') {
            config.histeq_plateau = std::max(0.01f, config.histeq_plateau - 0.05f);
            if (config.agc_mode == 1 && config.pipeline_mode != 2) apply_camera_settings();
            std::cout << "HistEq plateau: " << std::fixed << std::setprecision(2) << config.histeq_plateau << "\n";
        } else if (key == 't' || key == 'T') {
            config.show_temperature = !config.show_temperature;
            std::cout << "Temperature overlay: " << (config.show_temperature ? "ON" : "OFF") << "\n";
        } else if (key == 'f' || key == 'F') {
            config.show_fps = !config.show_fps;
            std::cout << "FPS overlay: " << (config.show_fps ? "ON" : "OFF") << "\n";
        } else if (key == 'c' || key == 'C') {
            config.show_center_temp = !config.show_center_temp;
            std::cout << "Center temp overlay: " << (config.show_center_temp ? "ON" : "OFF") << "\n";
        } else if (key == 'i' || key == 'I') {
            config.show_pipeline_info = !config.show_pipeline_info;
            std::cout << "Pipeline info overlay: " << (config.show_pipeline_info ? "ON" : "OFF") << "\n";
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // Cleanup
    std::cout << "Cleaning up...\n";
    cv::destroyAllWindows();

    if (camera_handle) {
        seekcamera_capture_session_stop(camera_handle);
    }
    // 'mgr' was declared in this scope; to destroy it we need it still visible:
    // Move mgr declaration outside if you want to destroy here; or rely on SDK cleanup on process exit.
    // To be explicit, let's re-open 'mgr' in a smaller scope:
    // (No-op here; if you keep mgr variable from earlier scope, call seekcamera_manager_destroy(&mgr))

    auto end_time = std::chrono::steady_clock::now();
    auto total_s  = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    std::cout << "\n=== Performance Statistics ===\n"
              << "Total frames processed: " << frames_total.load() << "\n"
              << "Total frames displayed: " << display_count.load() << "\n"
              << "Total runtime: " << total_s << " s\n";
    if (total_s > 0) {
        std::cout << "Avg processing FPS (callback): " << (frames_total.load() / total_s) << "\n"
                  << "Avg display FPS: " << (display_count.load() / total_s) << "\n";
    }
    std::cout << "Enhanced thermal camera viewer closed.\n";
    return 0;
}
