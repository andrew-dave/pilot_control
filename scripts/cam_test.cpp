#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    const std::string dev = "/dev/video1";          // 16 MP USB camera
    cv::VideoCapture cap(dev, cv::CAP_V4L2);        // Force V4L2 backend
    if (!cap.isOpened()) {
        std::cerr << "❌  Could not open " << dev << " with V4L2 backend\n";
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS,          30);

    std::cout << "✅  Camera opened (" << dev << ").  Press 'q' or ESC to exit.\n";
    std::cout << "    Actual FPS: " << cap.get(cv::CAP_PROP_FPS) << "\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::imshow("Live camera", frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }
    cap.release();
    cv::destroyAllWindows();
}