#include <iostream>
#include <Eigen/Dense>
#include <cmath>

int main() {
    // Test the rotation matrix for -30 degrees
    double angle = -0.5230; // -30 degrees in radians
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    
    std::cout << "Testing rotation matrix for " << (angle * 180.0 / M_PI) << " degrees" << std::endl;
    std::cout << "cos = " << cos_angle << ", sin = " << sin_angle << std::endl;
    
    // Create rotation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = cos_angle;
    transform(0, 2) = sin_angle;
    transform(2, 0) = -sin_angle;
    transform(2, 2) = cos_angle;
    
    std::cout << "Rotation matrix:" << std::endl;
    std::cout << transform << std::endl;
    
    // Test with a point that should be affected
    Eigen::Vector4f test_point(1.0, 0.0, 1.0, 1.0); // Point at (1, 0, 1)
    Eigen::Vector4f rotated_point = transform * test_point;
    
    std::cout << "Original point: (" << test_point(0) << ", " << test_point(1) << ", " << test_point(2) << ")" << std::endl;
    std::cout << "Rotated point:  (" << rotated_point(0) << ", " << rotated_point(1) << ", " << rotated_point(2) << ")" << std::endl;
    
    return 0;
} 