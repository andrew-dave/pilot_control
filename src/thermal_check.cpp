/**
 * @file thermal_check.cpp
 * @brief Headless Seek Thermal camera check — grabs one frame, saves PNG, reports pass/fail.
 *
 * Usage:
 *   thermal_check /path/to/output.png [timeout_seconds] [min_range_celsius]
 *
 * Exit codes:
 *   0 = PASS (frame captured, temperature range above threshold)
 *   1 = FAIL (timeout, no camera, or occluded — low temperature range)
 *   2 = ERROR (SDK / usage error)
 *
 * Stdout (machine-readable, one line):
 *   PASS range=8.7 min=18.2 max=26.9 center=22.1 w=320 h=240
 *   FAIL timeout
 *   FAIL range=0.3 min=22.1 max=22.4 center=22.2 w=320 h=240
 *   ERROR <message>
 */

#include <seekcamera/seekcamera.h>
#include <seekcamera/seekcamera_manager.h>
#include <seekcamera/seekcamera_frame.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>

// ─── Shared state between callbacks and main ───

struct ThermalResult {
    bool captured = false;
    float min_temp = 0.0f;
    float max_temp = 0.0f;
    float center_temp = 0.0f;
    float range = 0.0f;
    int width = 0;
    int height = 0;
    cv::Mat color_image;  // Colorized PNG (JET)
};

static std::mutex g_mutex;
static ThermalResult g_result;
static std::atomic<bool> g_frame_received(false);
static std::atomic<bool> g_camera_connected(false);
static std::atomic<bool> g_should_exit(false);

// ─── Frame callback ───

static void on_frame(seekcamera_t* /*cam*/, seekcamera_frame_t* cam_frame, void* /*user*/) {
    // Only capture the first frame
    if (g_frame_received.load()) return;

    seekcamera_frame_lock(cam_frame);

    seekframe_t* therm = nullptr;
    if (seekcamera_frame_get_frame_by_format(
            cam_frame, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &therm) == SEEKCAMERA_SUCCESS && therm) {

        const float* px = static_cast<const float*>(seekframe_get_data(therm));
        int w = static_cast<int>(seekframe_get_width(therm));
        int h = static_cast<int>(seekframe_get_height(therm));

        if (w > 0 && h > 0 && px) {
            cv::Mat thermal_mat(h, w, CV_32F, const_cast<float*>(px));

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(thermal_mat, &min_val, &max_val, &min_loc, &max_loc);
            float center = thermal_mat.at<float>(h / 2, w / 2);

            // Colorize for PNG output
            cv::Mat norm8;
            cv::normalize(thermal_mat, norm8, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::Mat colored;
            cv::applyColorMap(norm8, colored, cv::COLORMAP_JET);

            // Overlay temperature text
            cv::putText(colored, cv::format("Min: %.1fC", min_val),
                        {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
            cv::putText(colored, cv::format("Max: %.1fC", max_val),
                        {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
            cv::putText(colored, cv::format("Center: %.1fC", center),
                        {10, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
            cv::putText(colored, cv::format("Range: %.1fC", max_val - min_val),
                        {10, 100}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);

            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_result.captured = true;
                g_result.min_temp = static_cast<float>(min_val);
                g_result.max_temp = static_cast<float>(max_val);
                g_result.center_temp = center;
                g_result.range = static_cast<float>(max_val - min_val);
                g_result.width = w;
                g_result.height = h;
                g_result.color_image = colored.clone();
            }
            g_frame_received.store(true);
        }
    }

    seekcamera_frame_unlock(cam_frame);
}

// ─── Event callback ───

static void on_event(seekcamera_t* cam, seekcamera_manager_event_t ev,
                     seekcamera_error_t /*status*/, void* /*user*/) {
    switch (ev) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT: {
            g_camera_connected.store(true);

            seekcamera_register_frame_available_callback(cam, on_frame, nullptr);

            uint32_t fmt = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
            seekcamera_error_t err = seekcamera_capture_session_start(cam, fmt);
            if (err != SEEKCAMERA_SUCCESS) {
                std::cerr << "Failed to start capture session (err=" << err << ")" << std::endl;
            }
            break;
        }
        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            seekcamera_store_calibration_data(cam, nullptr, nullptr, nullptr);
            break;
        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
            seekcamera_capture_session_stop(cam);
            break;
        default:
            break;
    }
}

// ─── Signal handler for clean exit ───

static void signal_handler(int /*sig*/) {
    g_should_exit.store(true);
}

// ─── Main ───

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "ERROR usage: thermal_check <output_png> [timeout_s] [min_range_c]" << std::endl;
        return 2;
    }

    std::string output_path = argv[1];
    double timeout_s = (argc >= 3) ? std::atof(argv[2]) : 15.0;
    double min_range_c = (argc >= 4) ? std::atof(argv[3]) : 2.0;

    // Clamp timeout
    if (timeout_s <= 0 || timeout_s > 120) timeout_s = 15.0;
    if (min_range_c < 0) min_range_c = 2.0;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Create camera manager
    seekcamera_manager_t* mgr = nullptr;
    if (seekcamera_manager_create(&mgr, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cout << "ERROR failed_to_create_camera_manager" << std::endl;
        return 2;
    }

    seekcamera_manager_register_event_callback(mgr, on_event, nullptr);

    // Wait for a frame or timeout
    auto t0 = std::chrono::steady_clock::now();
    while (!g_should_exit.load()) {
        if (g_frame_received.load()) break;

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= timeout_s) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup SDK
    seekcamera_manager_destroy(&mgr);

    // Evaluate result
    if (!g_frame_received.load()) {
        if (!g_camera_connected.load()) {
            std::cout << "FAIL not_connected" << std::endl;
        } else {
            std::cout << "FAIL timeout" << std::endl;
        }
        return 1;
    }

    std::lock_guard<std::mutex> lock(g_mutex);

    // Save image
    if (!g_result.color_image.empty()) {
        cv::imwrite(output_path, g_result.color_image);
    }

    // Check occlusion threshold
    bool pass = g_result.range >= static_cast<float>(min_range_c);
    const char* status = pass ? "PASS" : "FAIL";

    std::cout << status
              << " range=" << cv::format("%.1f", g_result.range)
              << " min=" << cv::format("%.1f", g_result.min_temp)
              << " max=" << cv::format("%.1f", g_result.max_temp)
              << " center=" << cv::format("%.1f", g_result.center_temp)
              << " w=" << g_result.width
              << " h=" << g_result.height
              << std::endl;

    return pass ? 0 : 1;
}
