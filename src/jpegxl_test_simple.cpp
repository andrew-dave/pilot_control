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
  std::string output_dir = std::string(getenv("HOME")?getenv("HOME"):".") + "/jpegxl_test";
  fs::create_directories(output_dir);
  
  std::cout << "JPEG XL Test - Creating test images..." << std::endl;
  std::cout << "Output directory: " << output_dir << std::endl;
  
#if HAVE_JPEGXL
  std::cout << "✓ JPEG XL support enabled" << std::endl;
#else
  std::cout << "⚠ JPEG XL not available - will only save PNG" << std::endl;
#endif
  
  // Create test images
  std::vector<cv::Mat> test_images;
  
  // 1. Gradient image
  cv::Mat gradient(480, 640, CV_8UC3);
  for (int y = 0; y < gradient.rows; y++) {
    for (int x = 0; x < gradient.cols; x++) {
      gradient.at<cv::Vec3b>(y, x) = cv::Vec3b(
        (x * 255) / gradient.cols,           // Blue
        (y * 255) / gradient.rows,           // Green  
        ((x + y) * 255) / (gradient.cols + gradient.rows)  // Red
      );
    }
  }
  test_images.push_back(gradient);
  
  // 2. Random noise image
  cv::Mat noise(480, 640, CV_8UC3);
  cv::randu(noise, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
  test_images.push_back(noise);
  
  // 3. Checkerboard pattern
  cv::Mat checker(480, 640, CV_8UC3);
  int square_size = 40;
  for (int y = 0; y < checker.rows; y++) {
    for (int x = 0; x < checker.cols; x++) {
      bool is_white = ((x / square_size) + (y / square_size)) % 2 == 0;
      checker.at<cv::Vec3b>(y, x) = is_white ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
    }
  }
  test_images.push_back(checker);
  
  std::vector<std::string> image_names = {"gradient", "noise", "checkerboard"};
  
  std::string timestamp = timestampStr();
  
  for (size_t i = 0; i < test_images.size(); i++) {
    std::string stem = timestamp + "_" + image_names[i];
    
    // Save as PNG
    std::string png_path = output_dir + "/" + stem + ".png";
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9); // Maximum compression
    
    bool png_success = cv::imwrite(png_path, test_images[i], compression_params);
    
    // Save as JPEG XL (if available)
    std::string jxl_path = output_dir + "/" + stem + ".jxl";
    bool jxl_success = false;
    
#if HAVE_JPEGXL
    std::vector<uint8_t> compressed_data;
    if (compress_opencv_to_jxl(test_images[i], compressed_data, 3, 1.0)) {
      std::ofstream file(jxl_path, std::ios::binary);
      file.write(reinterpret_cast<const char*>(compressed_data.data()), compressed_data.size());
      jxl_success = true;
    }
#endif
    
    std::cout << "\n" << image_names[i] << " image:" << std::endl;
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
  }
  
  std::cout << "\n✓ Test completed! Check the files in: " << output_dir << std::endl;
  
  return 0;
}
