#include "seekcamera.h"
#include "seekcamera_manager.h"
#include "seekframe.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

// Configuration parameters
struct ThermalConfig {
    int color_palette = 0;  // 0=JET, 1=HOT, 2=COOL, 3=RAINBOW, 4=VIRIDIS, 5=INFERNO
    int agc_mode = 1;       // 0=Linear, 1=Histogram Equalization, 2=Adaptive
    int target_fps = 30;    // Target display FPS
    bool show_temperature = true;
    bool show_fps = true;
    bool show_center_temp = true;
    int colormap_alpha = 255; // 0-255 for transparency
};

// Global variables for ultra-optimized performance
std::mutex frame_mutex;
cv::Mat latest_frame;
std::atomic<bool> new_frame_available(false);
std::atomic<bool> should_exit(false);
std::atomic<bool> camera_connected(false);
std::string window_name = "Seek Thermal Camera - Optimized";

// Performance counters
std::atomic<uint64_t> frame_count(0);
std::atomic<uint64_t> display_count(0);
std::chrono::steady_clock::time_point start_time;
std::chrono::steady_clock::time_point fps_start_time;

// Configuration
ThermalConfig config;

// Color palette mappings
std::vector<cv::ColormapTypes> colormaps = {
    cv::COLORMAP_JET,      // 0
    cv::COLORMAP_HOT,      // 1
    cv::COLORMAP_COOL,     // 2
    cv::COLORMAP_RAINBOW,  // 3
    cv::COLORMAP_VIRIDIS,  // 4
    cv::COLORMAP_INFERNO   // 5
};

std::vector<std::string> colormap_names = {
    "JET", "HOT", "COOL", "RAINBOW", "VIRIDIS", "INFERNO"
};

// Ultra-fast frame processing with minimal latency
static void on_frame(seekcamera_t* cam, seekcamera_frame_t* cam_frame, void* user) {
    seekcamera_frame_lock(cam_frame);

    // Get thermography float frame for temperature data
    seekframe_t* therm = NULL;
    if (seekcamera_frame_get_frame_by_format(cam_frame,
            SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &therm) == SEEKCAMERA_SUCCESS && therm) {

        const float* px = (const float*)seekframe_get_data(therm);
        size_t w = seekframe_get_width(therm), h = seekframe_get_height(therm);

        // Ultra-fast processing pipeline
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            
            // Direct processing without unnecessary copies
            cv::Mat thermal_mat(h, w, CV_32F, (void*)px);
            
            // Apply AGC (Automatic Gain Control) based on mode
            cv::Mat processed_mat;
            switch (config.agc_mode) {
                case 0: // Linear
                    cv::normalize(thermal_mat, processed_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
                    break;
                case 1: // Histogram Equalization
                    cv::normalize(thermal_mat, processed_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
                    cv::equalizeHist(processed_mat, processed_mat);
                    break;
                case 2: // Adaptive
                    cv::normalize(thermal_mat, processed_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
                    clahe->apply(processed_mat, processed_mat);
                    break;
            }
            
            // Apply selected colormap
            cv::Mat colored;
            cv::applyColorMap(processed_mat, colored, colormaps[config.color_palette]);
            
            // Add overlays only if enabled
            if (config.show_temperature || config.show_fps || config.show_center_temp) {
                // Get temperature statistics
                double min_val, max_val, center_temp;
                cv::Point min_loc, max_loc;
                cv::minMaxLoc(thermal_mat, &min_val, &max_val, &min_loc, &max_loc);
                center_temp = px[h/2 * w + w/2];
                
                // Add temperature information
                if (config.show_temperature) {
                    std::stringstream ss;
                    ss << "Min: " << std::fixed << std::setprecision(1) << min_val << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                    
                    ss.str("");
                    ss << "Max: " << std::fixed << std::setprecision(1) << max_val << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                }
                
                // Add center temperature
                if (config.show_center_temp) {
                    std::stringstream ss;
                    ss << "Center: " << std::fixed << std::setprecision(1) << center_temp << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                }
                
                // Add FPS counter
                if (config.show_fps) {
                    frame_count++;
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start_time).count();
                    
                    if (elapsed >= 1000) { // Update FPS every second
                        double fps = (frame_count.load() * 1000.0) / elapsed;
                        std::stringstream ss;
                        ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
                        cv::putText(colored, ss.str(), cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                        
                        // Reset counters
                        frame_count = 0;
                        fps_start_time = now;
                    }
                }
                
                // Add configuration info
                std::string config_info = "Palette: " + colormap_names[config.color_palette] + 
                                        " | AGC: " + std::to_string(config.agc_mode);
                cv::putText(colored, config_info, cv::Point(10, h - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
            }
            
            // Store the processed frame
            latest_frame = colored.clone();
            new_frame_available = true;
        }
    }

    seekcamera_frame_unlock(cam_frame);
}

static void on_event(seekcamera_t* cam, seekcamera_manager_event_t ev,
                     seekcamera_error_t status, void* user) {
    if (ev == SEEKCAMERA_MANAGER_EVENT_CONNECT) {
        std::cout << "Camera connected successfully!" << std::endl;
        camera_connected = true;
        
        // Register frame callback
        seekcamera_register_frame_available_callback(cam, on_frame, NULL);

        // Request thermography format
        uint32_t fmt = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
        seekcamera_capture_session_start(cam, fmt);
        
        std::cout << "Thermal imaging started. Controls:" << std::endl;
        std::cout << "  ESC - Exit" << std::endl;
        std::cout << "  1-6 - Change color palette (JET/HOT/COOL/RAINBOW/VIRIDIS/INFERNO)" << std::endl;
        std::cout << "  Q/W - Change AGC mode (Linear/Histogram/Adaptive)" << std::endl;
        std::cout << "  T - Toggle temperature display" << std::endl;
        std::cout << "  F - Toggle FPS display" << std::endl;
        std::cout << "  C - Toggle center temperature" << std::endl;
        
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR) {
        std::cout << "Camera ready for pairing..." << std::endl;
        seekcamera_store_calibration_data(cam, NULL, NULL, NULL);
        
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_DISCONNECT) {
        std::cout << "Camera disconnected." << std::endl;
        camera_connected = false;
        seekcamera_capture_session_stop(cam);
    }
}

void print_usage() {
    std::cout << "Usage: ./seek_thermal_viewer [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -p <0-5>    Color palette (0=JET, 1=HOT, 2=COOL, 3=RAINBOW, 4=VIRIDIS, 5=INFERNO)" << std::endl;
    std::cout << "  -a <0-2>    AGC mode (0=Linear, 1=Histogram, 2=Adaptive)" << std::endl;
    std::cout << "  -f <fps>    Target FPS (default: 30)" << std::endl;
    std::cout << "  -t          Disable temperature display" << std::endl;
    std::cout << "  -s          Disable FPS display" << std::endl;
    std::cout << "  -c          Disable center temperature" << std::endl;
    std::cout << "  -h          Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Runtime controls:" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;
    std::cout << "  1-6 - Change color palette" << std::endl;
    std::cout << "  Q/W - Change AGC mode" << std::endl;
    std::cout << "  T/F/C - Toggle displays" << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-p" && i + 1 < argc) {
            config.color_palette = std::stoi(argv[++i]);
            if (config.color_palette < 0 || config.color_palette > 5) {
                std::cerr << "Invalid palette. Use 0-5." << std::endl;
                return 1;
            }
        } else if (arg == "-a" && i + 1 < argc) {
            config.agc_mode = std::stoi(argv[++i]);
            if (config.agc_mode < 0 || config.agc_mode > 2) {
                std::cerr << "Invalid AGC mode. Use 0-2." << std::endl;
                return 1;
            }
        } else if (arg == "-f" && i + 1 < argc) {
            config.target_fps = std::stoi(argv[++i]);
            if (config.target_fps < 1 || config.target_fps > 120) {
                std::cerr << "Invalid FPS. Use 1-120." << std::endl;
                return 1;
            }
        } else if (arg == "-t") {
            config.show_temperature = false;
        } else if (arg == "-s") {
            config.show_fps = false;
        } else if (arg == "-c") {
            config.show_center_temp = false;
        }
    }
    
    std::cout << "Starting Seek Thermal Camera Viewer..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Color Palette: " << colormap_names[config.color_palette] << std::endl;
    std::cout << "  AGC Mode: " << config.agc_mode << std::endl;
    std::cout << "  Target FPS: " << config.target_fps << std::endl;
    std::cout << "  Temperature Display: " << (config.show_temperature ? "ON" : "OFF") << std::endl;
    std::cout << "  FPS Display: " << (config.show_fps ? "ON" : "OFF") << std::endl;
    std::cout << "  Center Temp: " << (config.show_center_temp ? "ON" : "OFF") << std::endl;
    
    // Initialize performance tracking
    start_time = std::chrono::steady_clock::now();
    fps_start_time = start_time;
    
    // Create OpenCV window
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
    seekcamera_manager_t* mgr = NULL;
    if (seekcamera_manager_create(&mgr, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager" << std::endl;
        return 1;
    }
    
    seekcamera_manager_register_event_callback(mgr, on_event, NULL);
    std::cout << "Camera manager created. Waiting for camera connection..." << std::endl;
    
    // Ultra-optimized main display loop
    auto last_display_time = std::chrono::steady_clock::now();
    const auto display_interval = std::chrono::milliseconds(1000 / config.target_fps);
    
    while (!should_exit.load()) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Display frame rate limiting
        if (current_time - last_display_time >= display_interval) {
            if (new_frame_available.load()) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty()) {
                    cv::imshow(window_name, latest_frame);
                    new_frame_available = false;
                    last_display_time = current_time;
                    display_count++;
                }
            }
        }
        
        // Handle keyboard input
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) { // ESC key
            std::cout << "ESC pressed. Exiting..." << std::endl;
            should_exit = true;
            break;
        } else if (key >= '1' && key <= '6') {
            // Change color palette
            config.color_palette = key - '1';
            std::cout << "Changed to " << colormap_names[config.color_palette] << " palette" << std::endl;
        } else if (key == 'q') {
            // Previous AGC mode
            config.agc_mode = (config.agc_mode - 1 + 3) % 3;
            std::cout << "AGC Mode: " << config.agc_mode << std::endl;
        } else if (key == 'w') {
            // Next AGC mode
            config.agc_mode = (config.agc_mode + 1) % 3;
            std::cout << "AGC Mode: " << config.agc_mode << std::endl;
        } else if (key == 't') {
            config.show_temperature = !config.show_temperature;
            std::cout << "Temperature display: " << (config.show_temperature ? "ON" : "OFF") << std::endl;
        } else if (key == 'f') {
            config.show_fps = !config.show_fps;
            std::cout << "FPS display: " << (config.show_fps ? "ON" : "OFF") << std::endl;
        } else if (key == 'c') {
            config.show_center_temp = !config.show_center_temp;
            std::cout << "Center temperature: " << (config.show_center_temp ? "ON" : "OFF") << std::endl;
        }
        
        // Minimal sleep for CPU efficiency
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    // Cleanup
    std::cout << "Cleaning up..." << std::endl;
    cv::destroyAllWindows();
    seekcamera_manager_destroy(&mgr);
    
    // Print performance statistics
    auto end_time = std::chrono::steady_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    
    std::cout << "\n=== Performance Statistics ===" << std::endl;
    std::cout << "Total frames processed: " << frame_count.load() << std::endl;
    std::cout << "Total frames displayed: " << display_count.load() << std::endl;
    std::cout << "Total runtime: " << total_time << " seconds" << std::endl;
    if (total_time > 0) {
        std::cout << "Average processing FPS: " << frame_count.load() / total_time << std::endl;
        std::cout << "Average display FPS: " << display_count.load() / total_time << std::endl;
    }
    std::cout << "Thermal camera viewer closed successfully." << std::endl;
    
    return 0;
}
