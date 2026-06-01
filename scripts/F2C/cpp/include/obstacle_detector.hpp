/**
 * @file obstacle_detector.hpp
 * @brief Automatic obstacle detection from a 3D point cloud + driven path.
 *
 * This is a C++ port of `scripts/test_scripts/test_obstacle_detection.py`.
 * It uses the driven path to sample "footprint ground" points, then either:
 * - fits a single plane with RANSAC, or
 * - estimates a smooth local height field from nearby ground samples.
 *
 * It then extracts obstacle candidates above/below the ground band, removes
 * outliers, clusters with DBSCAN, merges nearby clusters, and polygonizes each
 * merged group.
 *
 * This port tracks the evolving Python reference implementation, including:
 * - grid-based polygonization with optional coarser contour extraction
 * - conservative smoothing (rolling-disk closing) with optional hole preservation
 * - micro-obstacle preservation (tiny but dense)
 */

#pragma once

#include "coverage_pipeline.hpp"

#include <string>
#include <vector>

namespace f2c_cpp {

enum class ObstaclePolygonMode {
    Auto,
    Hull,
    Grid,
};

enum class GroundModelMode {
    SinglePlane,
    LocalHeightField,
    PropagatedGrid,
    GroundZGradientGrid,
};

enum class ObstacleDetectionMethod {
    PathGroundAuto,
    PatchworkRawBundle,
    ClothSimulationFilter,
};

enum class GridEmptyCellPolicy {
    ConservativeBlocked,
    PropagateAcrossUnknown,
};

struct ObstacleDetectionParams {
    // Robot dimensions (metres) - defaults match the Python script.
    double robot_length_m = 0.31;
    double robot_width_m = 0.28;
    double footprint_margin_m = 0.10;

    // Ground detection
    ObstacleDetectionMethod detection_method = ObstacleDetectionMethod::PathGroundAuto;
    GroundModelMode ground_model_mode = GroundModelMode::SinglePlane;
    std::string source_path;
    double ground_z_max = 0.0;
    int ransac_iters = 300;
    double ransac_thresh_m = 0.03;
    double ground_band_m = 0.05;
    double local_ground_cell_m = 0.20;
    double local_ground_radius_m = 1.00;
    int local_ground_knn = 32;
    int local_ground_min_pts = 8;
    double local_ground_slope_reg = 0.05;
    double scope_margin_m = 0.25;
    double propagation_max_slope = 0.35;
    double ground_support_band_down = 0.03;
    double ground_support_band_up = 0.03;
    GridEmptyCellPolicy empty_cell_policy = GridEmptyCellPolicy::ConservativeBlocked;
    int min_support_points = 2;
    double wheel_radius_m = 0.072;
    double traversable_step_height_ratio = 0.70;
    double traversable_step_height_m = -1.0;  // if <= 0, derive from wheel radius * ratio
    double overhang_clearance_m = 0.18;
    double raw_cell_low_quantile = 0.05;
    double raw_cell_high_quantile = 0.95;
    double support_cluster_top_quantile = 0.90;
    double csf_cloth_resolution_m = 0.35;
    int csf_max_iterations = 2000;
    double csf_classification_threshold_m = 0.03;
    int csf_rigidness = 4;
    bool csf_slope_processing = false;
    double csf_max_obstacle_clearance_m = 0.50;
    bool csf_trail_footprint_cleanup = true;
    double csf_trail_cleanup_margin_m = 0.150;
    bool csf_pre_sor_enabled = true;
    int csf_pre_sor_k = 20;
    double csf_pre_sor_std = 1.5;

    // Obstacle extraction
    double obstacle_z_max = 0.30;
    double trough_depth_m = 0.05;
    int outlier_k = 20;
    double outlier_std = 1.5;

    // Clustering
    double cluster_eps_m = 0.07;
    int cluster_min_pts = 10;

    // Polygonization / chaining (defaults match current Python script)
    ObstaclePolygonMode polygon_mode = ObstaclePolygonMode::Grid;
    double grid_cell_m = 0.09;
    double contour_cell_m = -1.0;    // if < 0 => 2 * grid_cell_m
    double inflate_radius_m = 0.0;  // default: disabled
    double smooth_radius_m = -1.0;  // if < 0 => 2 * grid_cell_m (kept for parity)

    // Post-process smoothing (rolling-disk closing)
    double geom_smooth_radius_m = 1.0;
    int geom_smooth_segs = 16;  // (Shapely-only in Python; kept for parity)
    bool preserve_holes = true;
    double preserve_holes_min_area_m2 = 0.5;

    // AUTO mode heuristic
    double hollow_ratio_thresh = 0.35;
    double min_contour_area_m2 = 1e-4;

    // Cluster merging (metres)
    double merge_distance_m = 0.50;

    // Micro obstacles (tiny but dense) - enabled by default in Python
    bool micro_enable = true;
    double micro_max_span_m = 0.01;
    double micro_min_size_m = 0.01;
    double micro_margin_m = 0.002;
    int micro_min_pts = 22;
    double micro_min_density_pts_per_m2 = 200000.0;
    double micro_noise_eps_m = 0.025;
};

struct ObstacleDetectionStats {
    size_t input_points = 0;
    size_t roi_points = 0;
    size_t path_poses = 0;
    size_t footprint_ground_points = 0;
    size_t ground_points_band = 0;
    size_t raw_obstacle_candidates = 0;
    size_t obstacle_points_after_outlier = 0;
    size_t anchor_cells = 0;
    size_t propagated_ground_cells = 0;
    size_t blocked_unknown_cells = 0;
    size_t measured_obstacle_cells = 0;
    size_t traversable_overhang_cells = 0;
    size_t filled_empty_cells = 0;
    size_t high_gradient_edges = 0;
    int clusters_found = 0;
    int groups_merged = 0;
    int obstacle_shapes = 0;
    int total_holes = 0;

    // Plane: n·p + d = 0
    double plane_nx = 0.0;
    double plane_ny = 0.0;
    double plane_nz = 1.0;
    double plane_d = 0.0;
    double gradient_threshold = 0.0;
};

struct ObstacleDetectionResult {
    bool success = false;
    std::string error_message;
    std::vector<Obstacle2D> obstacles;
    std::vector<Obstacle2D> debug_grid_cells;
    ObstacleDetectionStats stats;
    PointCloudPtr csf_ground_cloud;
    PointCloudPtr csf_nonground_cloud;
    PointCloudPtr csf_sor_nonground_cloud;
    std::vector<Obstacle2D> csf_clearance_point_cells;
    std::vector<Obstacle2D> csf_occupancy_obstacles;
};

using ObstacleCancelCallback = std::function<bool()>;

/**
 * @brief Set a cooperative cancellation callback for long obstacle-detection runs.
 */
void setObstacleCancelCallback(ObstacleCancelCallback callback);

/**
 * @brief Main obstacle-detection dispatcher.
 *
 * The default behavior remains the current path-ground-based auto detector.
 * Additional non-default methods (such as Patchwork bundle loading) dispatch
 * here without changing the default pipeline.
 */
ObstacleDetectionResult detectObstacles(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& driven_path,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params = {});

/**
 * @brief Detect obstacles using AUTO mode (hull by default, grid-with-holes when hollow).
 *
 * @param cloud Point cloud currently loaded in the UI (map frame).
 * @param driven_path Driven odometry trail poses (x,y,heading). Used to sample footprint ground.
 * @param roi_or_boundary Optional polygon to restrict detection (legacy; UI may choose to post-filter instead).
 * @param params Algorithm parameters (defaults match Python script).
 */
ObstacleDetectionResult detectObstaclesAuto(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& driven_path,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params = {});

}  // namespace f2c_cpp

