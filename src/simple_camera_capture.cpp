#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <iomanip>
#include <sstream>

// JPEG XL includes (conditional)
#if HAVE_JPEGXL
#include <jxl/encode.h>
#include <jxl/thread_parallel_runner.h>
#include <jxl/color_encoding.h>
#endif

namespace fs = std::filesystem;

// ==================== JPEG XL Compression Helper ====================
#if HAVE_JPEGXL
static bool compress_opencv_to_jxl(const cv::Mat& image, std::vector<uint8_t>& compressed_data, 
                                  int effort = 3, double /*distance*/ = 1.0) {
  if (image.empty() || image.channels() != 3) return false;
  
  // Create encoder
  JxlEncoder* encoder = JxlEncoderCreate(nullptr);
  if (!encoder) return false;
  
  // Set up parallel runner for better performance (limit to 4 threads for speed)
  int num_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
  void* runner = JxlThreadParallelRunnerCreate(nullptr, num_threads);
  JxlEncoderSetParallelRunner(encoder, JxlThreadParallelRunner, runner);
  
  // Configure basic info
  JxlBasicInfo basic_info;
  JxlEncoderInitBasicInfo(&basic_info);
  basic_info.xsize = image.cols;
  basic_info.ysize = image.rows;
  basic_info.bits_per_sample = 8;
  basic_info.num_color_channels = 3;
  basic_info.alpha_bits = 0;
  basic_info.alpha_exponent_bits = 0;
  basic_info.uses_original_profile = JXL_FALSE;
  
  JxlEncoderSetBasicInfo(encoder, &basic_info);
  
  // Set color encoding (sRGB)
  JxlColorEncoding color_encoding = {};
  JxlColorEncodingSetToSRGB(&color_encoding, JXL_FALSE);
  JxlEncoderSetColorEncoding(encoder, &color_encoding);
  
  // Configure frame settings
  JxlEncoderFrameSettings* frame_settings = JxlEncoderFrameSettingsCreate(encoder, nullptr);
  
  // Set compression parameters for speed
  JxlEncoderFrameSettingsSetOption(frame_settings, JXL_ENC_FRAME_SETTING_EFFORT, effort);
  
  // Use BGR directly (avoid conversion) - JPEG XL can handle BGR
  JxlPixelFormat pixel_format = {3, JXL_TYPE_UINT8, JXL_NATIVE_ENDIAN, 0};
  
  // Add image frame directly (no BGR->RGB conversion needed)
  if (JxlEncoderAddImageFrame(frame_settings, &pixel_format, 
                             image.data, image.total() * image.elemSize()) != JXL_ENC_SUCCESS) {
    JxlThreadParallelRunnerDestroy(runner);
    JxlEncoderDestroy(encoder);
    return false;
  }
  
  JxlEncoderCloseInput(encoder);
  
  // Process output with smaller initial buffer for speed
  compressed_data.resize(32 * 1024); // Smaller initial buffer for faster allocation
  uint8_t* next_out = compressed_data.data();
  size_t avail_out = compressed_data.size();
  
  JxlEncoderStatus status = JXL_ENC_NEED_MORE_OUTPUT;
  while (status == JXL_ENC_NEED_MORE_OUTPUT) {
    status = JxlEncoderProcessOutput(encoder, &next_out, &avail_out);
    if (status == JXL_ENC_NEED_MORE_OUTPUT) {
      size_t offset = next_out - compressed_data.data();
      compressed_data.resize(compressed_data.size() * 2);
      next_out = compressed_data.data() + offset;
      avail_out = compressed_data.size() - offset;
    }
  }
  
  compressed_data.resize(next_out - compressed_data.data());
  
  // Cleanup
  JxlThreadParallelRunnerDestroy(runner);
  JxlEncoderDestroy(encoder);
  
  return status == JXL_ENC_SUCCESS;
}
#else
// Fallback: JPEG XL not available, return false
static bool compress_opencv_to_jxl(const cv::Mat& image, std::vector<uint8_t>& compressed_data, 
                                  int effort = 3, double distance = 1.0) {
  (void)image; (void)compressed_data; (void)effort; (void)distance; // Suppress unused warnings
  return false;
}
#endif

std::string timestampStr(){
  const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm,&t);
#else
  localtime_r(&t,&tm);
#endif
  char buf[32]; std::strftime(buf,sizeof(buf),"%Y%m%d_%H%M%S",&tm);
  return std::string(buf);
}

int main() {
  // Create output directory
  std::string output_dir = std::string(getenv("HOME")?getenv("HOME"):".") + "/camera_capture";
  fs::create_directories(output_dir);
  
  // Open camera (try different indices)
  cv::VideoCapture cap;
  int camera_index = 0;
  
  std::cout << "Trying to open camera..." << std::endl;
  for (int i = 0; i < 5; i++) {
    cap.open(i);
    if (cap.isOpened()) {
      camera_index = i;
      std::cout << "✓ Camera opened on index " << i << std::endl;
      break;
    }
  }
  
  if (!cap.isOpened()) {
    std::cerr << "✗ Failed to open any camera!" << std::endl;
    return -1;
  }
  
  // Set camera properties
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap.set(cv::CAP_PROP_FPS, 30);
  
  std::cout << "Camera properties:" << std::endl;
  std::cout << "  Width: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
  std::cout << "  Height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
  std::cout << "  FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
  
  std::cout << "\nControls:" << std::endl;
  std::cout << "  C - Capture image" << std::endl;
  std::cout << "  Q - Quit" << std::endl;
  std::cout << "  Output directory: " << output_dir << std::endl;
  
#if HAVE_JPEGXL
  std::cout << "✓ JPEG XL support enabled" << std::endl;
#else
  std::cout << "⚠ JPEG XL not available - will only save PNG" << std::endl;
#endif
  
  cv::Mat frame;
  int frame_count = 0;
  
  while (true) {
    cap >> frame;
    if (frame.empty()) {
      std::cerr << "Failed to capture frame!" << std::endl;
      break;
    }
    
    // Display the frame
    cv::imshow("Camera Capture - Press C to capture, Q to quit", frame);
    
    char key = cv::waitKey(1) & 0xFF;
    
    if (key == 'q' || key == 'Q') {
      std::cout << "Quitting..." << std::endl;
      break;
    }
    else if (key == 'c' || key == 'C') {
      frame_count++;
      std::string timestamp = timestampStr();
      std::ostringstream filename;
      filename << timestamp << "_frame" << std::setfill('0') << std::setw(3) << frame_count;
      
      // Save as PNG (always works)
      std::string png_path = output_dir + "/" + filename.str() + ".png";
      std::vector<int> compression_params;
      compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9); // Maximum compression
      
      bool png_success = cv::imwrite(png_path, frame, compression_params);
      
      // Save as JPEG XL (if available)
      std::string jxl_path = output_dir + "/" + filename.str() + ".jxl";
      bool jxl_success = false;
      
#if HAVE_JPEGXL
      std::vector<uint8_t> compressed_data;
      if (compress_opencv_to_jxl(frame, compressed_data, 3, 1.0)) {
        std::ofstream file(jxl_path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
        jxl_success = true;
      }
#endif
      
      std::cout << "Captured frame " << frame_count << ":" << std::endl;
      std::cout << "  PNG: " << (png_success ? "✓ " + png_path : "✗ FAILED") << std::endl;
      std::cout << "  JXL: " << (jxl_success ? "✓ " + jxl_path : "✗ FAILED") << std::endl;
      
      if (png_success) {
        // Get file sizes for comparison
        auto png_size = fs::file_size(png_path);
        std::cout << "  PNG size: " << png_size << " bytes" << std::endl;
        
        if (jxl_success) {
          auto jxl_size = fs::file_size(jxl_path);
          std::cout << "  JXL size: " << jxl_size << " bytes" << std::endl;
          double ratio = (double)jxl_size / png_size * 100.0;
          std::cout << "  JXL/PNG ratio: " << std::fixed << std::setprecision(1) << ratio << "%" << std::endl;
        }
      }
      std::cout << std::endl;
    }
  }
  
  cap.release();
  cv::destroyAllWindows();
  
  return 0;
}
