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
    int pipeline_mode = 2;  // 0=IMAGE_LITE, 1=IMAGE_LEGACY, 2=IMAGE_SEEKVISION
    int agc_mode = 0;       // 0=Linear, 1=HistEQ
    int target_fps = 30;    // Target display FPS
    bool show_temperature = true;
    bool show_fps = true;
    bool show_center_temp = true;
    bool show_pipeline_info = true;
    float histeq_plateau = 0.1f; // HistEQ AGC plateau value
};

// Global variables for ultra-optimized performance
std::mutex frame_mutex;
cv::Mat latest_frame;
std::atomic<bool> new_frame_available(false);
std::atomic<bool> should_exit(false);
std::atomic<bool> camera_connected(false);
std::string window_name = "Seek Thermal Camera - Enhanced";

// Performance counters
std::atomic<uint64_t> frame_count(0);
std::atomic<uint64_t> display_count(0);
std::chrono::steady_clock::time_point start_time;
std::chrono::steady_clock::time_point fps_start_time;

// Camera handle for SDK functions
seekcamera_t* camera_handle = nullptr;

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

// Pipeline mode mappings
std::vector<std::string> pipeline_names = {
    "IMAGE_LITE", "IMAGE_LEGACY", "IMAGE_SEEKVISION"
};

// AGC mode mappings
std::vector<std::string> agc_names = {
    "LINEAR", "HISTEQ"
};

// Apply SDK pipeline and AGC settings
void apply_camera_settings() {
    if (!camera_handle) return;
    
    // Set pipeline mode
    seekcamera_pipeline_mode_t pipeline_mode = (seekcamera_pipeline_mode_t)config.pipeline_mode;
    seekcamera_error_t result = seekcamera_set_pipeline_mode(camera_handle, pipeline_mode);
    if (result == SEEKCAMERA_SUCCESS) {
        std::cout << "Pipeline mode set to: " << pipeline_names[config.pipeline_mode] << std::endl;
    } else {
        std::cout << "Failed to set pipeline mode: " << result << std::endl;
    }
    
    // Set AGC mode (not available in IMAGE_SEEKVISION mode)
    if (config.pipeline_mode != 2) { // Not IMAGE_SEEKVISION
        seekcamera_agc_mode_t agc_mode = (seekcamera_agc_mode_t)config.agc_mode;
        result = seekcamera_set_agc_mode(camera_handle, agc_mode);
        if (result == SEEKCAMERA_SUCCESS) {
            std::cout << "AGC mode set to: " << agc_names[config.agc_mode] << std::endl;
        } else {
            std::cout << "Failed to set AGC mode: " << result << std::endl;
        }
    } else {
        std::cout << "AGC mode: Auto-controlled by IMAGE_SEEKVISION" << std::endl;
    }
    
    // Set HistEQ plateau if using HistEQ AGC (not available in IMAGE_SEEKVISION mode)
    if (config.agc_mode == 1 && config.pipeline_mode != 2) {
        result = seekcamera_set_histeq_agc_plateau(camera_handle, config.histeq_plateau);
        if (result == SEEKCAMERA_SUCCESS) {
            std::cout << "HistEQ plateau set to: " << config.histeq_plateau << std::endl;
        }
    }
}

// Get current camera settings
void get_camera_settings() {
    if (!camera_handle) return;
    
    // Get current pipeline mode
    seekcamera_pipeline_mode_t current_pipeline;
    seekcamera_error_t result = seekcamera_get_pipeline_mode(camera_handle, &current_pipeline);
    if (result == SEEKCAMERA_SUCCESS) {
        std::cout << "Current pipeline mode: " << seekcamera_pipeline_mode_get_str(current_pipeline) << std::endl;
        // Don't override user's configured pipeline mode
        // config.pipeline_mode = (int)current_pipeline;
    }
    
    // Get current AGC mode
    seekcamera_agc_mode_t current_agc;
    result = seekcamera_get_agc_mode(camera_handle, &current_agc);
    if (result == SEEKCAMERA_SUCCESS) {
        std::cout << "Current AGC mode: " << (current_agc == SEEKCAMERA_AGC_MODE_LINEAR ? "LINEAR" : "HISTEQ") << std::endl;
        // Don't override user's configured AGC mode
        // config.agc_mode = (int)current_agc;
    }
    
    // Get HistEQ plateau if applicable
    if (config.agc_mode == 1) {
        float current_plateau;
        result = seekcamera_get_histeq_agc_plateau(camera_handle, &current_plateau);
        if (result == SEEKCAMERA_SUCCESS) {
            std::cout << "Current HistEQ plateau: " << current_plateau << std::endl;
            config.histeq_plateau = current_plateau;
        }
    }
}

// Ultra-fast frame processing with SDK integration
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
            
            // The SDK handles AGC processing, so we just normalize for display
            cv::Mat processed_mat;
            cv::normalize(thermal_mat, processed_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
            
            // Apply selected colormap
            cv::Mat colored;
            cv::applyColorMap(processed_mat, colored, colormaps[config.color_palette]);
            
            // Add overlays only if enabled
            if (config.show_temperature || config.show_fps || config.show_center_temp || config.show_pipeline_info) {
                // Get temperature statistics
                double min_val, max_val, center_temp;
                cv::Point min_loc, max_loc;
                cv::minMaxLoc(thermal_mat, &min_val, &max_val, &min_loc, &max_loc);
                center_temp = px[h/2 * w + w/2];
                
                int y_offset = 25;
                
                // Add temperature information
                if (config.show_temperature) {
                    std::stringstream ss;
                    ss << "Min: " << std::fixed << std::setprecision(1) << min_val << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                    y_offset += 25;
                    
                    ss.str("");
                    ss << "Max: " << std::fixed << std::setprecision(1) << max_val << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                    y_offset += 25;
                }
                
                // Add center temperature
                if (config.show_center_temp) {
                    std::stringstream ss;
                    ss << "Center: " << std::fixed << std::setprecision(1) << center_temp << "C";
                    cv::putText(colored, ss.str(), cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                    y_offset += 25;
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
                        cv::putText(colored, ss.str(), cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                        y_offset += 25;
                        
                        // Reset counters
                        frame_count = 0;
                        fps_start_time = now;
                    }
                }
                
                // Add pipeline and AGC info
                if (config.show_pipeline_info) {
                    std::string pipeline_info = "Pipeline: " + pipeline_names[config.pipeline_mode] + 
                                              " | AGC: " + agc_names[config.agc_mode];
                    cv::putText(colored, pipeline_info, cv::Point(10, h - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
                    
                    // Add colormap info
                    std::string colormap_info = "Colormap: " + colormap_names[config.color_palette];
                    cv::putText(colored, colormap_info, cv::Point(10, h - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
                }
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
        camera_handle = cam; // Store camera handle for SDK functions
        
        // Register frame callback
        seekcamera_register_frame_available_callback(cam, on_frame, NULL);

        // Request thermography format
        uint32_t fmt = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
        seekcamera_capture_session_start(cam, fmt);
        
        // Get current camera settings
        get_camera_settings();
        
        // Apply our configuration
        apply_camera_settings();
        
        std::cout << "Thermal imaging started. Controls:" << std::endl;
        std::cout << "  ESC - Exit" << std::endl;
        std::cout << "  1-6 - Change color palette (JET/HOT/COOL/RAINBOW/VIRIDIS/INFERNO)" << std::endl;
        std::cout << "  P - Change pipeline mode (IMAGE_LITE/IMAGE_LEGACY/IMAGE_SEEKVISION)" << std::endl;
        std::cout << "  A - Change AGC mode (LINEAR/HISTEQ)" << std::endl;
        std::cout << "  +/- - Adjust HistEQ plateau value" << std::endl;
        std::cout << "  T/F/C/I - Toggle temperature/FPS/center/pipeline info displays" << std::endl;
        
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR) {
        std::cout << "Camera ready for pairing..." << std::endl;
        seekcamera_store_calibration_data(cam, NULL, NULL, NULL);
        
    } else if (ev == SEEKCAMERA_MANAGER_EVENT_DISCONNECT) {
        std::cout << "Camera disconnected." << std::endl;
        camera_connected = false;
        camera_handle = nullptr;
        seekcamera_capture_session_stop(cam);
    }
}

void print_usage() {
    std::cout << "Usage: ./seek_thermal_viewer_enhanced [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -p <0-5>    Color palette (0=JET, 1=HOT, 2=COOL, 3=RAINBOW, 4=VIRIDIS, 5=INFERNO)" << std::endl;
    std::cout << "  -m <0-2>    Pipeline mode (0=IMAGE_LITE, 1=IMAGE_LEGACY, 2=IMAGE_SEEKVISION)" << std::endl;
    std::cout << "  -a <0-1>    AGC mode (0=LINEAR, 1=HISTEQ)" << std::endl;
    std::cout << "  -f <fps>    Target FPS (default: 30)" << std::endl;
    std::cout << "  -t          Disable temperature display" << std::endl;
    std::cout << "  -s          Disable FPS display" << std::endl;
    std::cout << "  -c          Disable center temperature" << std::endl;
    std::cout << "  -i          Disable pipeline info display" << std::endl;
    std::cout << "  -h          Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Runtime controls:" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;
    std::cout << "  1-6 - Change color palette" << std::endl;
    std::cout << "  P - Change pipeline mode" << std::endl;
    std::cout << "  A - Change AGC mode" << std::endl;
    std::cout << "  +/- - Adjust HistEQ plateau" << std::endl;
    std::cout << "  T/F/C/I - Toggle displays" << std::endl;
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
        } else if (arg == "-m" && i + 1 < argc) {
            config.pipeline_mode = std::stoi(argv[++i]);
            if (config.pipeline_mode < 0 || config.pipeline_mode > 2) {
                std::cerr << "Invalid pipeline mode. Use 0-2." << std::endl;
                return 1;
            }
        } else if (arg == "-a" && i + 1 < argc) {
            config.agc_mode = std::stoi(argv[++i]);
            if (config.agc_mode < 0 || config.agc_mode > 1) {
                std::cerr << "Invalid AGC mode. Use 0-1." << std::endl;
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
        } else if (arg == "-i") {
            config.show_pipeline_info = false;
        }
    }
    
    std::cout << "Starting Enhanced Seek Thermal Camera Viewer..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Color Palette: " << colormap_names[config.color_palette] << std::endl;
    std::cout << "  Pipeline Mode: " << pipeline_names[config.pipeline_mode] << std::endl;
    std::cout << "  AGC Mode: " << agc_names[config.agc_mode] << std::endl;
    std::cout << "  Target FPS: " << config.target_fps << std::endl;
    std::cout << "  Temperature Display: " << (config.show_temperature ? "ON" : "OFF") << std::endl;
    std::cout << "  FPS Display: " << (config.show_fps ? "ON" : "OFF") << std::endl;
    std::cout << "  Center Temp: " << (config.show_center_temp ? "ON" : "OFF") << std::endl;
    std::cout << "  Pipeline Info: " << (config.show_pipeline_info ? "ON" : "OFF") << std::endl;
    
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
        } else if (key == 'p' || key == 'P') {
            // Change pipeline mode
            config.pipeline_mode = (config.pipeline_mode + 1) % 3;
            apply_camera_settings();
        } else if (key == 'a' || key == 'A') {
            // Change AGC mode
            config.agc_mode = (config.agc_mode + 1) % 2;
            apply_camera_settings();
        } else if (key == '+' || key == '=') {
            // Increase HistEQ plateau
            config.histeq_plateau = std::min(1.0f, config.histeq_plateau + 0.05f);
            if (config.agc_mode == 1) apply_camera_settings();
        } else if (key == '-') {
            // Decrease HistEQ plateau
            config.histeq_plateau = std::max(0.01f, config.histeq_plateau - 0.05f);
            if (config.agc_mode == 1) apply_camera_settings();
        } else if (key == 't' || key == 'T') {
            config.show_temperature = !config.show_temperature;
            std::cout << "Temperature display: " << (config.show_temperature ? "ON" : "OFF") << std::endl;
        } else if (key == 'f' || key == 'F') {
            config.show_fps = !config.show_fps;
            std::cout << "FPS display: " << (config.show_fps ? "ON" : "OFF") << std::endl;
        } else if (key == 'c' || key == 'C') {
            config.show_center_temp = !config.show_center_temp;
            std::cout << "Center temperature: " << (config.show_center_temp ? "ON" : "OFF") << std::endl;
        } else if (key == 'i' || key == 'I') {
            config.show_pipeline_info = !config.show_pipeline_info;
            std::cout << "Pipeline info: " << (config.show_pipeline_info ? "ON" : "OFF") << std::endl;
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
    std::cout << "Enhanced thermal camera viewer closed successfully." << std::endl;
    
    return 0;
}
