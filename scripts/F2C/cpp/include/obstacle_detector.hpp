/**
 * @file obstacle_detector.hpp
 * @brief Automatic obstacle detection from a 3D point cloud + driven path.
 *
 * This is a C++ port of `scripts/test_scripts/test_obstacle_detection.py`.
 * It uses the driven path to sample "footprint ground" points, fits a plane
 * with RANSAC, extracts obstacle candidates above/below the ground band,
 * removes outliers, clusters with DBSCAN, merges nearby clusters, and then
 * polygonizes each merged group.
 *
 * Polygonization mode: AUTO (fast). Uses convex hull unless the cluster looks
 * hollow, in which case it uses occupancy-grid contour polygonization that
 * supports holes.
 */

#pragma once

#include "coverage_pipeline.hpp"

#include <string>
#include <vector>

namespace f2c_cpp {

struct ObstacleDetectionParams {
    // Robot dimensions (metres) - defaults match the Python script.
    double robot_length_m = 0.31;
    double robot_width_m = 0.28;
    double footprint_margin_m = 0.10;

    // Ground detection
    double ground_z_max = 0.0;
    int ransac_iters = 300;
    double ransac_thresh_m = 0.03;
    double ground_band_m = 0.05;

    // Obstacle extraction
    double obstacle_z_max = 0.30;
    double trough_depth_m = 0.05;
    int outlier_k = 20;
    double outlier_std = 1.5;

    // Clustering
    double cluster_eps_m = 0.08;
    int cluster_min_pts = 8;

    // AUTO polygonization (hull vs grid-with-holes)
    double grid_cell_m = 0.03;
    double inflate_radius_m = -1.0;  // if < 0 => 0.5 * max(robot_length, robot_width)
    double hollow_ratio_thresh = 0.35;
    double min_contour_area_m2 = 0.02;

    // Cluster merging (metres)
    double merge_distance_m = 0.50;
};

struct ObstacleDetectionStats {
    size_t input_points = 0;
    size_t roi_points = 0;
    size_t path_poses = 0;
    size_t footprint_ground_points = 0;
    size_t ground_points_band = 0;
    size_t raw_obstacle_candidates = 0;
    size_t obstacle_points_after_outlier = 0;
    int clusters_found = 0;
    int groups_merged = 0;
    int obstacle_shapes = 0;
    int total_holes = 0;

    // Plane: n·p + d = 0
    double plane_nx = 0.0;
    double plane_ny = 0.0;
    double plane_nz = 1.0;
    double plane_d = 0.0;
};

struct ObstacleDetectionResult {
    bool success = false;
    std::string error_message;
    std::vector<Obstacle2D> obstacles;
    ObstacleDetectionStats stats;
};

/**
 * @brief Detect obstacles using AUTO mode (hull by default, grid-with-holes when hollow).
 *
 * @param cloud Filtered point cloud currently loaded in the UI (map frame).
 * @param driven_path Driven odometry trail poses (x,y,heading). Used to sample footprint ground.
 * @param roi_or_boundary Optional polygon to restrict detection (ROI preferred, else boundary).
 * @param params Algorithm parameters (defaults match Python script).
 */
ObstacleDetectionResult detectObstaclesAuto(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& driven_path,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params = {});

}  // namespace f2c_cpp

