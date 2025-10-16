// seek_thermal_simple.cpp
// Simplified Seek Thermal Camera Viewer
// - IMAGE_SEEKVISION pipeline mode only
// - JET color palette only
// - Essential functionality only

#include "seekcamera.h"
#include "seekcamera_manager.h"
#include "seekcamera_frame.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <atomic>

// Global variables
static std::mutex frame_mutex;
static cv::Mat latest_frame;
static std::atomic<bool> new_frame_available(false);
static std::atomic<bool> should_exit(false);
static std::string window_name = "Seek Thermal - SeekVision + JET";

// Frame callback - process thermal data
static void on_frame(seekcamera_t* cam, seekcamera_frame_t* cam_frame, void* user) {
    (void)cam; (void)user;
    
    seekcamera_frame_lock(cam_frame);

    seekframe_t* therm = nullptr;
    if (seekcamera_frame_get_frame_by_format(
            cam_frame, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &therm) == SEEKCAMERA_SUCCESS && therm) {

        const float* px = static_cast<const float*>(seekframe_get_data(therm));
        size_t w = seekframe_get_width(therm);
        size_t h = seekframe_get_height(therm);

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            
            // Create thermal image from float data
            cv::Mat thermal_mat(h, w, CV_32F, (void*)px);
            
            // Normalize to 8-bit for display
            cv::Mat processed_mat;
            cv::normalize(thermal_mat, processed_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
            
            // Apply JET colormap
            cv::Mat colored;
            cv::applyColorMap(processed_mat, colored, cv::COLORMAP_JET);
            
            // Add basic temperature info
            double min_val, max_val, center_temp;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(thermal_mat, &min_val, &max_val, &min_loc, &max_loc);
            center_temp = thermal_mat.at<float>(h/2, w/2);
            
            // Overlay temperature readings
            cv::putText(colored, "Min: " + cv::format("%.1fC", min_val),
                       {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(colored, "Max: " + cv::format("%.1fC", max_val),
                       {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            cv::putText(colored, "Center: " + cv::format("%.1fC", center_temp),
                       {10, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
            
            // Pipeline info
            cv::putText(colored, "Pipeline: IMAGE_SEEKVISION | Colormap: WHITE_HOT",
                       {10, static_cast<int>(h) - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200,200,200}, 1);
            
            // Store processed frame
            latest_frame = colored.clone();
            new_frame_available = true;
        }
    }

    seekcamera_frame_unlock(cam_frame);
}

// Event callback - handle camera connection
static void on_event(seekcamera_t* cam, seekcamera_manager_event_t ev,
                     seekcamera_error_t status, void* user) {
    (void)status; (void)user;

    switch (ev) {
        case SEEKCAMERA_MANAGER_EVENT_CONNECT: {
            std::cout << "Camera connected successfully!" << std::endl;
            
            // Register frame callback
            seekcamera_register_frame_available_callback(cam, on_frame, nullptr);

            // Start capture session with thermography format
            uint32_t fmt = SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT;
            seekcamera_error_t err = seekcamera_capture_session_start(cam, fmt);
            if (err != SEEKCAMERA_SUCCESS) {
                std::cerr << "Failed to start capture session (err=" << err << ")" << std::endl;
                return;
            }

            // Set pipeline mode to IMAGE_SEEKVISION
            err = seekcamera_set_pipeline_mode(cam, SEEKCAMERA_IMAGE_SEEKVISION);
            if (err == SEEKCAMERA_SUCCESS) {
                std::cout << "Pipeline mode set to: IMAGE_SEEKVISION" << std::endl;
            } else {
                std::cout << "Failed to set pipeline mode (err=" << err << ")" << std::endl;
            }

            // Set color palette to JET (using WHITE_HOT as fallback if JET not available)
            err = seekcamera_set_color_palette(cam, SEEKCAMERA_COLOR_PALETTE_WHITE_HOT);
            if (err == SEEKCAMERA_SUCCESS) {
                std::cout << "Color palette set to: WHITE_HOT" << std::endl;
            } else {
                std::cout << "Failed to set color palette (err=" << err << ")" << std::endl;
            }
            
            std::cout << "Thermal imaging started with SeekVision optimization!" << std::endl;
            std::cout << "Press ESC to exit." << std::endl;
            break;
        }

        case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
            std::cout << "Camera ready for pairing..." << std::endl;
            seekcamera_store_calibration_data(cam, nullptr, nullptr, nullptr);
            break;

        case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
            std::cout << "Camera disconnected." << std::endl;
            seekcamera_capture_session_stop(cam);
            break;

        default:
            break;
    }
}

int main() {
    std::cout << "Starting Simplified Seek Thermal Camera Viewer..." << std::endl;
    std::cout << "Configuration: IMAGE_SEEKVISION + JET Colormap" << std::endl;
    
    // Create OpenCV window
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
    // Create camera manager
    seekcamera_manager_t* mgr = nullptr;
    if (seekcamera_manager_create(&mgr, SEEKCAMERA_IO_TYPE_USB) != SEEKCAMERA_SUCCESS) {
        std::cerr << "Failed to create camera manager" << std::endl;
        return 1;
    }
    
    // Register event callback
    seekcamera_manager_register_event_callback(mgr, on_event, nullptr);
    std::cout << "Camera manager created. Waiting for camera connection..." << std::endl;
    
    // Main display loop
    while (!should_exit.load()) {
        // Display frame if available
        if (new_frame_available.load()) {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty()) {
                cv::imshow(window_name, latest_frame);
                new_frame_available = false;
            }
        }
        
        // Handle keyboard input
        int key = cv::waitKey(30) & 0xFF;
        if (key == 27) { // ESC key
            std::cout << "ESC pressed. Exiting..." << std::endl;
            should_exit = true;
            break;
        }
    }
    
    // Cleanup
    std::cout << "Cleaning up..." << std::endl;
    cv::destroyAllWindows();
    seekcamera_manager_destroy(&mgr);
    
    std::cout << "Simplified thermal camera viewer closed successfully." << std::endl;
    return 0;
}
