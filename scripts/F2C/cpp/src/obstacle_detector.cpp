/**
 * @file obstacle_detector.cpp
 * @brief Implementation of automatic obstacle detection (AUTO mode).
 */

#include "obstacle_detector.hpp"

#include "CSF.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <unordered_map>
#include <utility>

namespace f2c_cpp {

static ObstacleCancelCallback g_obstacleCancelCallback = nullptr;

void setObstacleCancelCallback(ObstacleCancelCallback callback) {
    g_obstacleCancelCallback = callback;
}

namespace {

static constexpr double kEps = 1e-12;
static constexpr double kLocalGroundSeedQuantile = 0.20;

static double pointSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b);

static double signedArea2D(const Polygon2D& ring) {
    if (ring.size() < 3) return 0.0;
    double a = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const auto& p0 = ring[i];
        const auto& p1 = ring[(i + 1) % ring.size()];
        a += (p0.x * p1.y - p1.x * p0.y);
    }
    return 0.5 * a;
}

static void ensureCCW(Polygon2D& ring) {
    if (signedArea2D(ring) < 0.0) {
        std::reverse(ring.begin(), ring.end());
    }
}

static void ensureCW(Polygon2D& ring) {
    if (signedArea2D(ring) > 0.0) {
        std::reverse(ring.begin(), ring.end());
    }
}

static double absArea2D(const Polygon2D& ring) {
    return std::abs(signedArea2D(ring));
}

static Polygon2D rectFromBBox(
    double xmin, double ymin, double xmax, double ymax,
    double min_size, double margin) {
    // Axis-aligned bbox rectangle with optional minimum size and margin.
    const double cx = 0.5 * (xmin + xmax);
    const double cy = 0.5 * (ymin + ymax);
    const double w = std::max(min_size, (xmax - xmin)) + 2.0 * margin;
    const double h = std::max(min_size, (ymax - ymin)) + 2.0 * margin;
    const double hw = 0.5 * w;
    const double hh = 0.5 * h;

    Polygon2D poly;
    poly.reserve(4);
    poly.emplace_back(cx - hw, cy - hh);
    poly.emplace_back(cx + hw, cy - hh);
    poly.emplace_back(cx + hw, cy + hh);
    poly.emplace_back(cx - hw, cy + hh);
    return poly;
}

static bool pointInPolyRayCast(const Point2D& p, const Polygon2D& poly) {
    // Ray casting; matches the Python script's simple implementation.
    if (poly.size() < 3) return false;
    bool inside = false;
    double x = p.x;
    double y = p.y;
    Point2D p0 = poly.back();
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        bool intersects = ((p1.y > y) != (p0.y > y)) &&
            (x < (p0.x - p1.x) * (y - p1.y) / ((p0.y - p1.y) + 1e-12) + p1.x);
        if (intersects) {
            inside = !inside;
        }
        p0 = p1;
    }
    return inside;
}

static bool pointInPolyOrWithinMargin(const Point2D& p, const Polygon2D& poly, double margin) {
    if (poly.size() < 3) {
        return false;
    }
    if (pointInPolyRayCast(p, poly)) {
        return true;
    }
    if (margin <= 1e-9) {
        return false;
    }
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& a = poly[i];
        const Point2D& b = poly[(i + 1) % poly.size()];
        if (pointSegmentDistance(p, a, b) <= margin) {
            return true;
        }
    }
    return false;
}

static Polygon2D effectiveScopePolygon(const Polygon2D* roi_or_boundary) {
    if (roi_or_boundary && roi_or_boundary->size() >= 3) {
        return *roi_or_boundary;
    }
    return {};
}

static PointCloudPtr filterCloudToPolygon(
    const PointCloudPtr& cloud,
    const Polygon2D& scope,
    double margin_m = 0.0) {
    if (!cloud) return PointCloudPtr(new PointCloud);
    if (scope.size() < 3) {
        return cloud;
    }
    PointCloudPtr out(new PointCloud);
    out->reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        if (pointInPolyOrWithinMargin(Point2D(pt.x, pt.y), scope, margin_m)) {
            out->push_back(pt);
        }
    }
    return out;
}

static std::vector<PathState> filterPathToPolygon(
    const std::vector<PathState>& path,
    const Polygon2D& scope,
    double margin_m = 0.0) {
    if (scope.size() < 3) return path;
    std::vector<PathState> out;
    out.reserve(path.size());
    for (const auto& st : path) {
        if (pointInPolyOrWithinMargin(st.point, scope, margin_m)) {
            out.push_back(st);
        }
    }
    // If the ROI removes everything, fall back to full path.
    if (out.empty()) return path;
    return out;
}

static bool fileExists(const std::string& path) {
    return !path.empty() && std::filesystem::exists(path);
}

static std::string replaceLastOccurrence(std::string value,
                                         const std::string& needle,
                                         const std::string& replacement) {
    const size_t pos = value.rfind(needle);
    if (pos == std::string::npos) {
        return value;
    }
    value.replace(pos, needle.size(), replacement);
    return value;
}

struct PatchworkBundlePaths {
    std::string source_path;
    std::string scores_path;
    std::string ground_path;
    std::string nonground_path;
};

static PatchworkBundlePaths derivePatchworkBundlePaths(const std::string& source_path) {
    PatchworkBundlePaths bundle;
    bundle.source_path = source_path;
    bundle.scores_path = source_path;
    bundle.ground_path = source_path;
    bundle.nonground_path = source_path;

    if (source_path.find("corrected_patchwork_scores_") != std::string::npos) {
        bundle.ground_path = replaceLastOccurrence(source_path, "corrected_patchwork_scores_", "corrected_patchwork_ground_");
        bundle.nonground_path = replaceLastOccurrence(source_path, "corrected_patchwork_scores_", "corrected_patchwork_nonground_");
        return bundle;
    }
    if (source_path.find("corrected_patchwork_ground_") != std::string::npos) {
        bundle.scores_path = replaceLastOccurrence(source_path, "corrected_patchwork_ground_", "corrected_patchwork_scores_");
        bundle.nonground_path = replaceLastOccurrence(source_path, "corrected_patchwork_ground_", "corrected_patchwork_nonground_");
        return bundle;
    }
    if (source_path.find("corrected_patchwork_nonground_") != std::string::npos) {
        bundle.scores_path = replaceLastOccurrence(source_path, "corrected_patchwork_nonground_", "corrected_patchwork_scores_");
        bundle.ground_path = replaceLastOccurrence(source_path, "corrected_patchwork_nonground_", "corrected_patchwork_ground_");
        return bundle;
    }

    if (source_path.find("patchwork_scores_") != std::string::npos) {
        bundle.ground_path = replaceLastOccurrence(source_path, "patchwork_scores_", "patchwork_ground_");
        bundle.nonground_path = replaceLastOccurrence(source_path, "patchwork_scores_", "patchwork_nonground_");
        return bundle;
    }
    if (source_path.find("patchwork_ground_") != std::string::npos) {
        bundle.scores_path = replaceLastOccurrence(source_path, "patchwork_ground_", "patchwork_scores_");
        bundle.nonground_path = replaceLastOccurrence(source_path, "patchwork_ground_", "patchwork_nonground_");
        return bundle;
    }
    if (source_path.find("patchwork_nonground_") != std::string::npos) {
        bundle.scores_path = replaceLastOccurrence(source_path, "patchwork_nonground_", "patchwork_scores_");
        bundle.ground_path = replaceLastOccurrence(source_path, "patchwork_nonground_", "patchwork_ground_");
        return bundle;
    }

    return bundle;
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloudFileXYZI(const std::string& path) {
    if (!fileExists(path)) {
        throw std::runtime_error("Patchwork score cloud not found: " + path);
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::string ext = path.substr(path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    int result = -1;
    if (ext == "pcd") {
        result = pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);
    } else if (ext == "ply") {
        result = pcl::io::loadPLYFile<pcl::PointXYZI>(path, *cloud);
    } else {
        throw std::runtime_error("Unsupported Patchwork score file format: " + ext);
    }

    if (result < 0 || cloud->empty()) {
        throw std::runtime_error("Failed to load Patchwork score cloud: " + path);
    }
    return cloud;
}

struct PlaneModel {
    double nx = 0.0;
    double ny = 0.0;
    double nz = 1.0;
    double d = 0.0;  // n·p + d = 0
};

static const char* groundModelName(GroundModelMode mode) {
    switch (mode) {
        case GroundModelMode::GroundZGradientGrid:
            return "ground_z_gradient_grid";
        case GroundModelMode::PropagatedGrid:
            return "propagated_grid";
        case GroundModelMode::LocalHeightField:
            return "local_height_field";
        case GroundModelMode::SinglePlane:
        default:
            return "single_plane";
    }
}

static double signedDist(const PlaneModel& pl, const pcl::PointXYZ& p) {
    return pl.nx * p.x + pl.ny * p.y + pl.nz * p.z + pl.d;
}

static PlaneModel fitPlaneRansac(const PointCloudPtr& ground_cloud, int iters, double thresh_m) {
    PlaneModel best;
    if (!ground_cloud || ground_cloud->size() < 3) {
        return best;
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist_idx(0, ground_cloud->size() - 1);

    size_t best_count = 0;
    const size_t n = ground_cloud->size();

    auto getPt = [&](size_t i) -> Eigen::Vector3d {
        const auto& p = ground_cloud->points[i];
        return Eigen::Vector3d(p.x, p.y, p.z);
    };

    for (int iter = 0; iter < iters; ++iter) {
        size_t i0 = dist_idx(rng);
        size_t i1 = dist_idx(rng);
        size_t i2 = dist_idx(rng);
        if (i0 == i1 || i0 == i2 || i1 == i2) {
            continue;
        }
        Eigen::Vector3d p0 = getPt(i0);
        Eigen::Vector3d p1 = getPt(i1);
        Eigen::Vector3d p2 = getPt(i2);
        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p0;
        Eigen::Vector3d nrm = v1.cross(v2);
        double norm = nrm.norm();
        if (norm < 1e-12) continue;
        nrm /= norm;
        if (nrm.z() < 0.0) nrm = -nrm;
        double d = -nrm.dot(p0);

        size_t count = 0;
        for (const auto& pt : ground_cloud->points) {
            double dd = std::abs(nrm.x() * pt.x + nrm.y() * pt.y + nrm.z() * pt.z + d);
            if (dd <= thresh_m) {
                count++;
            }
        }
        if (count > best_count) {
            best_count = count;
            best.nx = nrm.x();
            best.ny = nrm.y();
            best.nz = nrm.z();
            best.d = d;
        }
    }
    return best;
}

static double medianZ(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;
    std::vector<float> z;
    z.reserve(cloud->size());
    for (const auto& p : cloud->points) z.push_back(p.z);
    size_t mid = z.size() / 2;
    std::nth_element(z.begin(), z.begin() + mid, z.end());
    return static_cast<double>(z[mid]);
}

static PlaneModel flatPlaneAtZ(double z) {
    PlaneModel plane;
    plane.nx = 0.0;
    plane.ny = 0.0;
    plane.nz = 1.0;
    plane.d = -z;
    return plane;
}

static PlaneModel fallbackPlaneFromGround(
    const PointCloudPtr& ground_cloud,
    int ransac_iters,
    double ransac_thresh_m) {
    if (!ground_cloud || ground_cloud->empty()) {
        return PlaneModel{};
    }
    if (ground_cloud->size() < 20) {
        return flatPlaneAtZ(medianZ(ground_cloud));
    }
    return fitPlaneRansac(ground_cloud, ransac_iters, ransac_thresh_m);
}

struct GroundCellKey {
    int ix = 0;
    int iy = 0;

    bool operator==(const GroundCellKey& other) const {
        return ix == other.ix && iy == other.iy;
    }
};

struct GroundCellKeyHash {
    size_t operator()(const GroundCellKey& key) const {
        const uint64_t x = static_cast<uint32_t>(key.ix);
        const uint64_t y = static_cast<uint32_t>(key.iy);
        return static_cast<size_t>((x << 32) ^ y);
    }
};

struct GroundCellAccum {
    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t xy_count = 0;
    std::vector<float> z_values;
};

static size_t quantileIndex(size_t n, double q) {
    if (n == 0) {
        return 0;
    }
    q = std::clamp(q, 0.0, 1.0);
    return std::min(
        n - 1,
        static_cast<size_t>(std::floor(q * static_cast<double>(n - 1))));
}

static PointCloudPtr aggregateGroundSeedsQuantileXY(
    const PointCloudPtr& cloud,
    double cell_m,
    double quantile) {
    if (!cloud) return PointCloudPtr(new PointCloud);
    if (cloud->empty()) return PointCloudPtr(new PointCloud);
    if (cell_m <= kEps) {
        return cloud;
    }

    std::unordered_map<GroundCellKey, GroundCellAccum, GroundCellKeyHash> cells;
    cells.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        const int ix = static_cast<int>(std::floor(pt.x / cell_m));
        const int iy = static_cast<int>(std::floor(pt.y / cell_m));
        GroundCellAccum& cell = cells[GroundCellKey{ix, iy}];
        cell.sum_x += pt.x;
        cell.sum_y += pt.y;
        cell.xy_count++;
        cell.z_values.push_back(pt.z);
    }

    PointCloudPtr out(new PointCloud);
    out->reserve(cells.size());
    for (auto& kv : cells) {
        GroundCellAccum& cell = kv.second;
        if (cell.xy_count == 0 || cell.z_values.empty()) {
            continue;
        }
        const size_t qidx = quantileIndex(cell.z_values.size(), quantile);
        std::nth_element(cell.z_values.begin(), cell.z_values.begin() + qidx, cell.z_values.end());
        const double x = cell.sum_x / static_cast<double>(cell.xy_count);
        const double y = cell.sum_y / static_cast<double>(cell.xy_count);
        const double z = static_cast<double>(cell.z_values[qidx]);
        out->push_back(pcl::PointXYZ(static_cast<float>(x),
                                     static_cast<float>(y),
                                     static_cast<float>(z)));
    }
    return out;
}

struct LocalHeightFieldModel {
    PointCloudPtr xy_support;
    std::vector<double> z_support;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    PlaneModel fallback_plane;
    int knn = 32;
    int min_pts = 8;
    double radius_m = 1.0;
    double slope_reg = 0.05;
    bool prefer_global_plane = false;
    bool valid = false;
};

static double percentileAbsPlaneResidual(
    const PointCloudPtr& cloud,
    const PlaneModel& plane,
    double q) {
    if (!cloud || cloud->empty()) {
        return std::numeric_limits<double>::infinity();
    }

    q = std::clamp(q, 0.0, 1.0);
    std::vector<double> residuals;
    residuals.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        residuals.push_back(std::abs(signedDist(plane, pt)));
    }
    const size_t idx = std::min(
        residuals.size() - 1,
        static_cast<size_t>(std::floor(q * static_cast<double>(residuals.size() - 1))));
    std::nth_element(residuals.begin(), residuals.begin() + static_cast<std::ptrdiff_t>(idx), residuals.end());
    return residuals[idx];
}

static LocalHeightFieldModel buildLocalHeightFieldModel(
    const PointCloudPtr& ground_cloud,
    const ObstacleDetectionParams& params) {
    LocalHeightFieldModel model;
    model.knn = std::max(3, params.local_ground_knn);
    model.min_pts = std::max(3, params.local_ground_min_pts);
    model.radius_m = std::max(0.10, params.local_ground_radius_m);
    model.slope_reg = std::max(0.0, params.local_ground_slope_reg);
    PointCloudPtr seeds = aggregateGroundSeedsQuantileXY(
        ground_cloud, params.local_ground_cell_m, kLocalGroundSeedQuantile);
    const PointCloudPtr plane_support =
        (seeds && seeds->size() >= static_cast<size_t>(model.min_pts)) ? seeds : ground_cloud;
    model.fallback_plane = fallbackPlaneFromGround(
        plane_support, params.ransac_iters, params.ransac_thresh_m);

    const double planar_residual_gate_m = std::max(
        0.015, std::min(0.5 * params.ground_band_m, 1.5 * params.ransac_thresh_m));
    const double planar_p90_residual_m =
        percentileAbsPlaneResidual(plane_support, model.fallback_plane, 0.90);
    if (std::isfinite(planar_p90_residual_m) && planar_p90_residual_m <= planar_residual_gate_m) {
        model.prefer_global_plane = true;
        return model;
    }

    if (!seeds || seeds->size() < static_cast<size_t>(model.min_pts)) {
        return model;
    }

    model.xy_support.reset(new PointCloud);
    model.xy_support->reserve(seeds->size());
    model.z_support.reserve(seeds->size());
    for (const auto& pt : seeds->points) {
        model.xy_support->push_back(
            pcl::PointXYZ(pt.x, pt.y, 0.0f));
        model.z_support.push_back(static_cast<double>(pt.z));
    }

    if (model.xy_support->size() < static_cast<size_t>(model.min_pts)) {
        return model;
    }

    model.tree.setInputCloud(model.xy_support);
    model.valid = true;
    return model;
}

static bool estimateLocalGroundPlane(
    const LocalHeightFieldModel& model,
    double x,
    double y,
    PlaneModel* plane_out) {
    if (!plane_out) {
        return false;
    }
    if (model.prefer_global_plane) {
        *plane_out = model.fallback_plane;
        return false;
    }
    if (!model.valid || !model.xy_support || model.xy_support->empty()) {
        *plane_out = model.fallback_plane;
        return false;
    }

    if (static_cast<int>(model.xy_support->size()) < model.min_pts) {
        *plane_out = model.fallback_plane;
        return false;
    }

    std::vector<int> nn_idx;
    std::vector<float> nn_dist2;
    nn_idx.reserve(static_cast<size_t>(model.knn));
    nn_dist2.reserve(static_cast<size_t>(model.knn));
    const pcl::PointXYZ query(static_cast<float>(x), static_cast<float>(y), 0.0f);
    const int found = model.tree.radiusSearch(
        query, static_cast<float>(model.radius_m), nn_idx, nn_dist2, model.knn);
    if (found < model.min_pts) {
        *plane_out = model.fallback_plane;
        return false;
    }

    double support_scale = model.radius_m;
    if (found > 0) {
        support_scale = std::max(
            support_scale,
            std::sqrt(std::max(
                0.0, static_cast<double>(nn_dist2[static_cast<size_t>(found - 1)]))));
    }
    support_scale = std::max(support_scale, 0.10);
    const double sigma2 = std::max(support_scale * support_scale, 1e-6);

    Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    double sum_w = 0.0;
    for (int i = 0; i < found; ++i) {
        const int idx = nn_idx[static_cast<size_t>(i)];
        const auto& seed = model.xy_support->points[static_cast<size_t>(idx)];
        const double dx = static_cast<double>(seed.x) - x;
        const double dy = static_cast<double>(seed.y) - y;
        const double d2 = dx * dx + dy * dy;
        double w = std::exp(-0.5 * d2 / sigma2);
        w = std::max(w, 1e-3);

        const Eigen::Vector3d phi(dx, dy, 1.0);
        lhs += w * (phi * phi.transpose());
        rhs += w * phi * model.z_support[static_cast<size_t>(idx)];
        sum_w += w;
    }

    if (sum_w <= 1e-9) {
        *plane_out = model.fallback_plane;
        return false;
    }

    const double lambda = model.slope_reg * std::max(sum_w, 1.0);
    lhs(0, 0) += lambda;
    lhs(1, 1) += lambda;

    Eigen::LDLT<Eigen::Matrix3d> solver(lhs);
    if (solver.info() != Eigen::Success) {
        *plane_out = model.fallback_plane;
        return false;
    }

    const Eigen::Vector3d coeff = solver.solve(rhs);
    if (solver.info() != Eigen::Success || !coeff.allFinite()) {
        *plane_out = model.fallback_plane;
        return false;
    }

    const double a = coeff.x();
    const double b = coeff.y();
    const double c = coeff.z();
    const double norm = std::sqrt(1.0 + a * a + b * b);
    if (!std::isfinite(norm) || norm <= 1e-9) {
        *plane_out = model.fallback_plane;
        return false;
    }

    plane_out->nx = -a / norm;
    plane_out->ny = -b / norm;
    plane_out->nz = 1.0 / norm;
    plane_out->d = (a * x + b * y - c) / norm;
    return true;
}

static PointCloudPtr extractFootprintGround(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& path,
    double robot_length_m,
    double robot_width_m,
    double footprint_margin_m,
    double z_max,
    bool enforce_z_max = true) {
    if (!cloud || cloud->empty()) return PointCloudPtr(new PointCloud);
    if (path.empty()) return PointCloudPtr(new PointCloud);

    // Build KD-tree of path XY
    pcl::PointCloud<pcl::PointXYZ>::Ptr path_xy(new pcl::PointCloud<pcl::PointXYZ>);
    path_xy->reserve(path.size());
    for (const auto& st : path) {
        path_xy->push_back(pcl::PointXYZ(static_cast<float>(st.point.x),
                                         static_cast<float>(st.point.y),
                                         0.0f));
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> path_tree;
    path_tree.setInputCloud(path_xy);

    const double half_l = robot_length_m / 2.0 + footprint_margin_m;
    const double half_w = robot_width_m / 2.0 + footprint_margin_m;
    const double search_radius = std::hypot(half_l, half_w);
    const double dist_limit = search_radius * 1.5;
    const int K = std::min<int>(5, static_cast<int>(path.size()));

    PointCloudPtr ground(new PointCloud);
    ground->reserve(cloud->size() / 10);

    std::vector<int> nn_idx;
    std::vector<float> nn_dist2;
    nn_idx.resize(std::max(1, K));
    nn_dist2.resize(std::max(1, K));

    for (const auto& pt : cloud->points) {
        if (enforce_z_max && pt.z > z_max) continue;

        pcl::PointXYZ q(pt.x, pt.y, 0.0f);
        int found = path_tree.nearestKSearch(q, K, nn_idx, nn_dist2);
        if (found <= 0) continue;

        bool hit = false;
        for (int i = 0; i < found; ++i) {
            double d = std::sqrt(static_cast<double>(nn_dist2[i]));
            if (d > dist_limit) {
                continue;
            }
            const auto& pose = path[static_cast<size_t>(nn_idx[i])];
            double cx = pose.point.x;
            double cy = pose.point.y;
            double yaw = pose.heading;

            double dx = pt.x - cx;
            double dy = pt.y - cy;
            double c = std::cos(yaw);
            double s = std::sin(yaw);
            // Rotate by -yaw into footprint local frame
            double lx = c * dx + s * dy;
            double ly = -s * dx + c * dy;
            if (std::abs(lx) <= half_l && std::abs(ly) <= half_w) {
                hit = true;
                break;
            }
        }
        if (hit) {
            ground->push_back(pt);
        }
    }

    return ground;
}

enum class PropagatedCellState {
    Outside,
    Unknown,
    AnchorGround,
    PropagatedGround,
    MeasuredObstacle,
    BlockedUnknown,
};

struct ZClusterSummary {
    size_t begin = 0;
    size_t end = 0;
    size_t count = 0;
    double low = 0.0;
    double median = 0.0;
    double high = 0.0;
};

struct PropagatedGridCell {
    bool inside_scope = false;
    bool trail_covered = false;
    bool ground_z_valid = false;
    bool gradient_reachable = false;
    PropagatedCellState state = PropagatedCellState::Outside;
    std::vector<float> z_values;
    std::vector<float> anchor_z_values;
    int point_count = 0;
    double low_z = 0.0;
    double median_z = 0.0;
    double high_z = 0.0;
    double vertical_span = 0.0;
    double ground_z = 0.0;
    double confidence = 0.0;
    int interpolation_contributors = 0;
    double incoming_gradient = 0.0;
    bool incoming_gradient_valid = false;
    int incoming_from_x = -1;
    int incoming_from_y = -1;
    int support_point_count = 0;
    double support_base_z = 0.0;
    double support_top_z = 0.0;
    double support_span_z = 0.0;
    double first_non_ground_z = 0.0;
    double clearance_above_ground = 0.0;
    std::string reason;
};

struct PropagatedGrid {
    int w = 0;
    int h = 0;
    double xmin = 0.0;
    double ymin = 0.0;
    double cell = 0.10;
    double scope_margin_m = 0.0;
    Polygon2D scope;
    std::vector<PropagatedGridCell> cells;
};

struct PropagationEstimate {
    bool valid = false;
    double predicted_z = 0.0;
    double confidence = 0.0;
    int contributors = 0;
};

struct GridClassification {
    PropagatedCellState state = PropagatedCellState::Unknown;
    double ground_z = 0.0;
    double confidence = 0.0;
    bool traversable_overhang = false;
    int support_point_count = 0;
    double support_base_z = 0.0;
    double support_top_z = 0.0;
    double support_span_z = 0.0;
    double first_non_ground_z = 0.0;
    double clearance_above_ground = 0.0;
    std::string reason;
};

struct GroundSupportAnalysis {
    bool found_support = false;
    int support_point_count = 0;
    double support_base_z = 0.0;
    double support_top_z = 0.0;
    double support_span_z = 0.0;
    bool has_upper_cluster = false;
    double first_non_ground_z = 0.0;
    double clearance_above_ground = std::numeric_limits<double>::infinity();
};

struct PropagationCandidate {
    double priority = 0.0;
    int x = 0;
    int y = 0;
};

struct PropagationCandidateCompare {
    bool operator()(const PropagationCandidate& a, const PropagationCandidate& b) const {
        return a.priority < b.priority;
    }
};

static bool isGroundCellState(PropagatedCellState state) {
    return state == PropagatedCellState::AnchorGround ||
           state == PropagatedCellState::PropagatedGround;
}

static size_t propagatedGridIndex(const PropagatedGrid& grid, int x, int y) {
    return static_cast<size_t>(y) * static_cast<size_t>(grid.w) + static_cast<size_t>(x);
}

static bool propagatedGridInBounds(const PropagatedGrid& grid, int x, int y) {
    return x >= 0 && y >= 0 && x < grid.w && y < grid.h;
}

static Point2D propagatedGridCellCenter(const PropagatedGrid& grid, int x, int y) {
    return Point2D(
        grid.xmin + (static_cast<double>(x) + 0.5) * grid.cell,
        grid.ymin + (static_cast<double>(y) + 0.5) * grid.cell);
}

static bool pointInsideFootprintAtPose(
    const Point2D& point,
    const PathState& pose,
    double half_l,
    double half_w) {
    const double dx = point.x - pose.point.x;
    const double dy = point.y - pose.point.y;
    const double c = std::cos(pose.heading);
    const double s = std::sin(pose.heading);
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;
    return std::abs(lx) <= half_l && std::abs(ly) <= half_w;
}

static double quantileValueSorted(const std::vector<float>& sorted_values, double q) {
    if (sorted_values.empty()) {
        return 0.0;
    }
    return static_cast<double>(sorted_values[quantileIndex(sorted_values.size(), q)]);
}

static double quantileValueCopy(std::vector<float> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    const size_t idx = quantileIndex(values.size(), q);
    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(idx),
        values.end());
    return static_cast<double>(values[idx]);
}

static double lowestValueSorted(const std::vector<float>& sorted_values) {
    if (sorted_values.empty()) {
        return 0.0;
    }
    return static_cast<double>(sorted_values.front());
}

static double derivedTraversableStepHeight(const ObstacleDetectionParams& params) {
    if (params.traversable_step_height_m > 0.0) {
        return params.traversable_step_height_m;
    }
    return std::max(
        0.01,
        std::max(0.0, params.traversable_step_height_ratio) * std::max(0.0, params.wheel_radius_m));
}

struct CsfSegmentationResult {
    PointCloudPtr ground;
    PointCloudPtr nonground;
    size_t grid_cells = 0;
};

static CsfSegmentationResult segmentGroundClothSimulation(
    const PointCloudPtr& cloud,
    const ObstacleDetectionParams& params,
    const std::function<bool()>& abortIfCancelled) {
    CsfSegmentationResult result;
    result.ground.reset(new PointCloud);
    result.nonground.reset(new PointCloud);
    if (!cloud || cloud->empty()) {
        return result;
    }

    if (abortIfCancelled && abortIfCancelled()) {
        return result;
    }

    std::vector<csf::Point> csf_points;
    csf_points.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        csf_points.push_back(csf::Point{
            static_cast<double>(pt.x),
            static_cast<double>(pt.y),
            static_cast<double>(pt.z)});
    }

    CSF csf;
    csf.params.bSloopSmooth = params.csf_slope_processing;
    csf.params.time_step = 0.65;
    csf.params.cloth_resolution = std::max(0.005, params.csf_cloth_resolution_m);
    csf.params.class_threshold = std::max(0.0, params.csf_classification_threshold_m);
    csf.params.interations = std::max(1, params.csf_max_iterations);
    csf.params.rigidness = std::clamp(params.csf_rigidness, 1, 10);
    csf.setPointCloud(std::move(csf_points));

    std::vector<int> ground_indices;
    std::vector<int> nonground_indices;
    csf.do_filtering(ground_indices, nonground_indices, /*exportCloth=*/false);
    result.grid_cells = csf.size();

    result.ground->reserve(cloud->size());
    result.nonground->reserve(cloud->size() / 4);

    for (int idx : ground_indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < cloud->size()) {
            result.ground->push_back(cloud->points[static_cast<size_t>(idx)]);
        }
    }

    for (int idx : nonground_indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= cloud->size()) {
            continue;
        }
        result.nonground->push_back(cloud->points[static_cast<size_t>(idx)]);
    }

    return result;
}

static PointCloudPtr filterCsfNonGroundByClearance(
    const PointCloudPtr& nonground,
    const PointCloudPtr& ground,
    const ObstacleDetectionParams& params,
    const std::function<bool()>& abortIfCancelled) {
    PointCloudPtr out(new PointCloud);
    if (!nonground || nonground->empty() || !ground || ground->empty()) {
        return out;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_xy(new pcl::PointCloud<pcl::PointXYZ>);
    ground_xy->reserve(ground->size());
    std::vector<float> ground_z;
    ground_z.reserve(ground->size());
    for (const auto& pt : ground->points) {
        ground_xy->push_back(pcl::PointXYZ(pt.x, pt.y, 0.0f));
        ground_z.push_back(pt.z);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> ground_tree;
    ground_tree.setInputCloud(ground_xy);

    const double min_clearance = derivedTraversableStepHeight(params);
    const double max_clearance = std::max(min_clearance, params.csf_max_obstacle_clearance_m);
    out->reserve(nonground->size());

    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist2(1);
    size_t idx = 0;
    for (const auto& pt : nonground->points) {
        if ((idx++ & 0x1FFFu) == 0u && abortIfCancelled && abortIfCancelled()) {
            return out;
        }
        const pcl::PointXYZ query(pt.x, pt.y, 0.0f);
        if (ground_tree.nearestKSearch(query, 1, nn_idx, nn_dist2) <= 0) {
            continue;
        }
        const double clearance = static_cast<double>(pt.z - ground_z[static_cast<size_t>(nn_idx[0])]);
        if (clearance >= min_clearance && clearance <= max_clearance) {
            out->push_back(pt);
        }
    }
    return out;
}

static std::vector<PathState> densifyPathForFootprintCleanup(
    const std::vector<PathState>& path,
    double spacing_m) {
    if (path.size() < 2 || spacing_m <= 1e-6) {
        return path;
    }
    std::vector<PathState> out;
    out.reserve(path.size() * 2);
    out.push_back(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
        const auto& a = path[i - 1];
        const auto& b = path[i];
        const double dx = b.point.x - a.point.x;
        const double dy = b.point.y - a.point.y;
        const double dist = std::hypot(dx, dy);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / spacing_m)));
        const double heading = (dist > 1e-6) ? std::atan2(dy, dx) : a.heading;
        for (int s = 1; s <= steps; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(steps);
            PathState st;
            st.point.x = a.point.x + dx * t;
            st.point.y = a.point.y + dy * t;
            st.heading = heading;
            st.vx = std::cos(heading);
            st.vy = std::sin(heading);
            out.push_back(st);
        }
    }
    return out;
}

static PointCloudPtr removeTrailFootprintObstacleCandidates(
    const PointCloudPtr& candidates,
    const std::vector<PathState>& path,
    const ObstacleDetectionParams& params,
    size_t* removed_count,
    const std::function<bool()>& abortIfCancelled) {
    if (removed_count) {
        *removed_count = 0;
    }
    if (!candidates || candidates->empty()) {
        return PointCloudPtr(new PointCloud);
    }
    if (path.empty() || !params.csf_trail_footprint_cleanup) {
        return candidates;
    }

    const double margin = std::max(0.0, params.csf_trail_cleanup_margin_m);
    const double half_l = std::max(0.0, params.robot_length_m) / 2.0 + margin;
    const double half_w = std::max(0.0, params.robot_width_m) / 2.0 + margin;
    const double spacing = std::max(0.03, 0.5 * std::min(half_l, half_w));
    const std::vector<PathState> dense_path = densifyPathForFootprintCleanup(path, spacing);
    if (dense_path.empty()) {
        return candidates;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr path_xy(new pcl::PointCloud<pcl::PointXYZ>);
    path_xy->reserve(dense_path.size());
    for (const auto& st : dense_path) {
        path_xy->push_back(pcl::PointXYZ(
            static_cast<float>(st.point.x),
            static_cast<float>(st.point.y),
            0.0f));
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> path_tree;
    path_tree.setInputCloud(path_xy);

    const double search_radius = std::hypot(half_l, half_w);
    const int max_nn = std::min<int>(12, static_cast<int>(dense_path.size()));
    std::vector<int> nn_idx(std::max(1, max_nn));
    std::vector<float> nn_dist2(std::max(1, max_nn));

    PointCloudPtr out(new PointCloud);
    out->reserve(candidates->size());
    size_t removed = 0;
    size_t idx = 0;
    for (const auto& pt : candidates->points) {
        if ((idx++ & 0x1FFFu) == 0u && abortIfCancelled && abortIfCancelled()) {
            if (removed_count) {
                *removed_count = removed;
            }
            return out;
        }
        const pcl::PointXYZ query(pt.x, pt.y, 0.0f);
        const int found = path_tree.radiusSearch(query, static_cast<float>(search_radius), nn_idx, nn_dist2, max_nn);
        bool in_trail = false;
        for (int i = 0; i < found; ++i) {
            const int pi = nn_idx[static_cast<size_t>(i)];
            if (pi < 0 || pi >= static_cast<int>(dense_path.size())) {
                continue;
            }
            if (pointInsideFootprintAtPose(Point2D(pt.x, pt.y), dense_path[static_cast<size_t>(pi)], half_l, half_w)) {
                in_trail = true;
                break;
            }
        }
        if (in_trail) {
            removed++;
            continue;
        }
        out->push_back(pt);
    }
    if (removed_count) {
        *removed_count = removed;
    }
    return out;
}

static bool propagatedGridCellInsideScope(const PropagatedGrid& grid, int x, int y) {
    if (grid.scope.size() < 3) {
        return true;
    }
    const Point2D center = propagatedGridCellCenter(grid, x, y);
    if (pointInPolyOrWithinMargin(center, grid.scope, grid.scope_margin_m)) {
        return true;
    }
    const double half = 0.5 * grid.cell;
    const std::array<Point2D, 4> corners = {
        Point2D(center.x - half, center.y - half),
        Point2D(center.x + half, center.y - half),
        Point2D(center.x + half, center.y + half),
        Point2D(center.x - half, center.y + half),
    };
    for (const auto& corner : corners) {
        if (pointInPolyOrWithinMargin(corner, grid.scope, grid.scope_margin_m)) {
            return true;
        }
    }
    return false;
}

static void finalizePropagatedGridCellStats(
    PropagatedGridCell* cell,
    const ObstacleDetectionParams& params) {
    if (!cell) {
        return;
    }
    cell->point_count = static_cast<int>(cell->z_values.size());
    cell->support_point_count = 0;
    cell->support_base_z = 0.0;
    cell->support_top_z = 0.0;
    cell->support_span_z = 0.0;
    cell->first_non_ground_z = 0.0;
    cell->clearance_above_ground = 0.0;
    cell->reason.clear();
    if (cell->z_values.empty()) {
        cell->low_z = 0.0;
        cell->median_z = 0.0;
        cell->high_z = 0.0;
        cell->vertical_span = 0.0;
        return;
    }
    std::sort(cell->z_values.begin(), cell->z_values.end());
    const double low_q = std::clamp(params.raw_cell_low_quantile, 0.0, 0.50);
    const double high_q = std::clamp(params.raw_cell_high_quantile, 0.50, 1.0);
    cell->low_z = quantileValueSorted(cell->z_values, low_q);
    cell->median_z = quantileValueSorted(cell->z_values, 0.50);
    cell->high_z = quantileValueSorted(cell->z_values, high_q);
    cell->vertical_span = std::max(0.0, cell->high_z - cell->low_z);
}

static PropagatedGrid buildPropagatedGrid(
    const PointCloudPtr& cloud,
    const Polygon2D& scope,
    double cell_m,
    double scope_margin_m,
    const ObstacleDetectionParams& params) {
    PropagatedGrid grid;
    grid.cell = std::max(0.05, cell_m);
    grid.scope_margin_m = std::max(0.0, scope_margin_m);
    grid.scope = scope;
    if ((!cloud || cloud->empty()) && scope.size() < 3) {
        return grid;
    }

    double minx = 0.0;
    double miny = 0.0;
    double maxx = 0.0;
    double maxy = 0.0;
    bool have_bounds = false;
    auto expandBounds = [&](double x, double y) {
        if (!have_bounds) {
            minx = maxx = x;
            miny = maxy = y;
            have_bounds = true;
            return;
        }
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    };

    if (scope.size() >= 3) {
        for (const auto& p : scope) {
            expandBounds(p.x, p.y);
        }
        minx -= grid.scope_margin_m;
        miny -= grid.scope_margin_m;
        maxx += grid.scope_margin_m;
        maxy += grid.scope_margin_m;
    } else if (cloud) {
        for (const auto& pt : cloud->points) {
            expandBounds(pt.x, pt.y);
        }
    }

    if (!have_bounds) {
        return grid;
    }

    grid.xmin = minx;
    grid.ymin = miny;
    grid.w = std::max(1, static_cast<int>(std::ceil((maxx - minx) / grid.cell)) + 1);
    grid.h = std::max(1, static_cast<int>(std::ceil((maxy - miny) / grid.cell)) + 1);
    grid.cells.resize(static_cast<size_t>(grid.w) * static_cast<size_t>(grid.h));

    for (int y = 0; y < grid.h; ++y) {
        for (int x = 0; x < grid.w; ++x) {
            PropagatedGridCell& cell = grid.cells[propagatedGridIndex(grid, x, y)];
            cell.inside_scope = propagatedGridCellInsideScope(grid, x, y);
            cell.state = cell.inside_scope ? PropagatedCellState::Unknown : PropagatedCellState::Outside;
        }
    }

    if (cloud) {
        for (const auto& pt : cloud->points) {
            const int x = static_cast<int>(std::floor((pt.x - grid.xmin) / grid.cell));
            const int y = static_cast<int>(std::floor((pt.y - grid.ymin) / grid.cell));
            if (!propagatedGridInBounds(grid, x, y)) {
                continue;
            }
            PropagatedGridCell& cell = grid.cells[propagatedGridIndex(grid, x, y)];
            if (!cell.inside_scope) {
                continue;
            }
            cell.z_values.push_back(pt.z);
        }
    }

    for (auto& cell : grid.cells) {
        finalizePropagatedGridCellStats(&cell, params);
    }
    return grid;
}

static void markTrailCoveredCells(
    PropagatedGrid* grid,
    const std::vector<PathState>& path,
    double robot_length_m,
    double robot_width_m,
    double footprint_margin_m) {
    if (!grid || grid->w <= 0 || grid->h <= 0 || path.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr path_xy(new pcl::PointCloud<pcl::PointXYZ>);
    path_xy->reserve(path.size());
    for (const auto& st : path) {
        path_xy->push_back(
            pcl::PointXYZ(static_cast<float>(st.point.x), static_cast<float>(st.point.y), 0.0f));
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> path_tree;
    path_tree.setInputCloud(path_xy);

    const double half_l = robot_length_m / 2.0 + footprint_margin_m;
    const double half_w = robot_width_m / 2.0 + footprint_margin_m;
    const double search_radius = std::hypot(half_l, half_w);
    const double dist_limit = search_radius * 1.5;
    const int K = std::min<int>(5, static_cast<int>(path.size()));
    std::vector<int> nn_idx(std::max(1, K));
    std::vector<float> nn_dist2(std::max(1, K));

    for (int y = 0; y < grid->h; ++y) {
        for (int x = 0; x < grid->w; ++x) {
            auto& cell = grid->cells[propagatedGridIndex(*grid, x, y)];
            if (!cell.inside_scope) {
                continue;
            }
            const Point2D center = propagatedGridCellCenter(*grid, x, y);
            const double half_cell = 0.5 * grid->cell;
            const std::array<Point2D, 5> samples = {
                center,
                Point2D(center.x - half_cell, center.y - half_cell),
                Point2D(center.x + half_cell, center.y - half_cell),
                Point2D(center.x + half_cell, center.y + half_cell),
                Point2D(center.x - half_cell, center.y + half_cell),
            };
            const pcl::PointXYZ query(
                static_cast<float>(center.x),
                static_cast<float>(center.y),
                0.0f);
            const int found = path_tree.nearestKSearch(query, K, nn_idx, nn_dist2);
            for (int i = 0; i < found; ++i) {
                const double d = std::sqrt(static_cast<double>(nn_dist2[static_cast<size_t>(i)]));
                if (d > dist_limit) {
                    continue;
                }
                const auto& pose = path[static_cast<size_t>(nn_idx[static_cast<size_t>(i)])];
                bool covered = false;
                for (const auto& sample : samples) {
                    if (pointInsideFootprintAtPose(sample, pose, half_l, half_w)) {
                        covered = true;
                        break;
                    }
                }
                if (covered) {
                    cell.trail_covered = true;
                    break;
                }
            }
        }
    }
}

static std::vector<ZClusterSummary> buildZClusters(
    const std::vector<float>& sorted_values,
    double max_gap_m) {
    std::vector<ZClusterSummary> clusters;
    if (sorted_values.empty()) {
        return clusters;
    }
    max_gap_m = std::max(0.01, max_gap_m);
    size_t begin = 0;
    for (size_t i = 1; i <= sorted_values.size(); ++i) {
        const bool split =
            (i == sorted_values.size()) ||
            (static_cast<double>(sorted_values[i]) - static_cast<double>(sorted_values[i - 1]) > max_gap_m);
        if (!split) {
            continue;
        }
        ZClusterSummary cluster;
        cluster.begin = begin;
        cluster.end = i;
        cluster.count = i - begin;
        cluster.low = static_cast<double>(sorted_values[begin]);
        cluster.high = static_cast<double>(sorted_values[i - 1]);
        const size_t mid = begin + ((i - begin) / 2);
        cluster.median = static_cast<double>(sorted_values[mid]);
        clusters.push_back(cluster);
        begin = i;
    }
    return clusters;
}

static bool selectGroundSupportCluster(
    const PropagatedGridCell& cell,
    double predicted_z,
    const ObstacleDetectionParams& params,
    ZClusterSummary* cluster_out) {
    if (cell.z_values.empty() || !cluster_out) {
        return false;
    }
    const double support_low = predicted_z - std::max(0.0, params.ground_support_band_down);
    const double support_high = predicted_z + std::max(0.0, params.ground_support_band_up);
    const double cluster_gap_m = std::max(
        0.02, 0.5 * std::max(params.ground_support_band_down, params.ground_support_band_up));
    const auto clusters = buildZClusters(cell.z_values, cluster_gap_m);

    bool found = false;
    size_t best_count = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& cluster : clusters) {
        if (cluster.high < support_low || cluster.low > support_high) {
            continue;
        }
        const double distance = std::abs(cluster.median - predicted_z);
        if (!found || cluster.count > best_count ||
            (cluster.count == best_count && distance < best_distance)) {
            *cluster_out = cluster;
            best_count = cluster.count;
            best_distance = distance;
            found = true;
        }
    }
    return found;
}

static bool estimateObservedAnchorGroundZ(
    const PropagatedGridCell& cell,
    double* ground_z_out,
    double* confidence_out) {
    if (!ground_z_out || !confidence_out) {
        return false;
    }
    if (!cell.anchor_z_values.empty()) {
        *ground_z_out = static_cast<double>(
            *std::min_element(cell.anchor_z_values.begin(), cell.anchor_z_values.end()));
        *confidence_out = 10.0 + static_cast<double>(cell.anchor_z_values.size());
        return true;
    }
    if (!cell.z_values.empty()) {
        *ground_z_out = lowestValueSorted(cell.z_values);
        *confidence_out = 4.0 + std::min(6.0, 0.25 * static_cast<double>(cell.point_count));
        return true;
    }
    return false;
}

static GroundSupportAnalysis analyzeGroundSupport(
    const PropagatedGridCell& cell,
    double predicted_z,
    const ObstacleDetectionParams& params) {
    GroundSupportAnalysis analysis;
    if (cell.z_values.empty()) {
        return analysis;
    }

    ZClusterSummary support_cluster;
    if (!selectGroundSupportCluster(cell, predicted_z, params, &support_cluster)) {
        return analysis;
    }

    const int min_support = std::max(1, params.min_support_points);
    if (static_cast<int>(support_cluster.count) < min_support) {
        return analysis;
    }

    const std::vector<float> support_values(
        cell.z_values.begin() + static_cast<std::ptrdiff_t>(support_cluster.begin),
        cell.z_values.begin() + static_cast<std::ptrdiff_t>(support_cluster.end));
    if (support_values.empty()) {
        return analysis;
    }

    analysis.found_support = true;
    analysis.support_point_count = static_cast<int>(support_values.size());
    analysis.support_base_z = lowestValueSorted(support_values);
    analysis.support_top_z = quantileValueSorted(
        support_values,
        std::clamp(params.support_cluster_top_quantile, 0.50, 1.0));
    analysis.support_span_z = std::max(0.0, analysis.support_top_z - analysis.support_base_z);

    const double cluster_gap_m = std::max(
        0.02, 0.5 * std::max(params.ground_support_band_down, params.ground_support_band_up));
    const auto clusters = buildZClusters(cell.z_values, cluster_gap_m);
    for (const auto& cluster : clusters) {
        if (cluster.begin <= support_cluster.begin) {
            continue;
        }
        analysis.has_upper_cluster = true;
        analysis.first_non_ground_z = cluster.low;
        analysis.clearance_above_ground =
            std::max(0.0, analysis.first_non_ground_z - analysis.support_base_z);
        break;
    }
    return analysis;
}

static double planeZAt(const PlaneModel& plane, double x, double y) {
    if (std::abs(plane.nz) <= 1e-9) {
        return 0.0;
    }
    return -(plane.nx * x + plane.ny * y + plane.d) / plane.nz;
}

static PropagationEstimate estimateGroundFromNeighbours(
    const PropagatedGrid& grid,
    int x,
    int y,
    const ObstacleDetectionParams& params) {
    PropagationEstimate estimate;
    if (!propagatedGridInBounds(grid, x, y)) {
        return estimate;
    }

    struct WeightedNeighbour {
        double z = 0.0;
        double weight = 0.0;
        double distance = 0.0;
    };
    std::vector<WeightedNeighbour> support;
    support.reserve(8);

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (!propagatedGridInBounds(grid, nx, ny)) {
                continue;
            }
            const auto& neighbour = grid.cells[propagatedGridIndex(grid, nx, ny)];
            if (!isGroundCellState(neighbour.state)) {
                continue;
            }
            const double base_weight = (dx == 0 || dy == 0) ? 1.0 : 0.5;
            double confidence = std::max(0.25, neighbour.confidence);
            if (neighbour.state == PropagatedCellState::AnchorGround) {
                confidence *= 10.0;
            }
            support.push_back(WeightedNeighbour{
                neighbour.ground_z,
                base_weight * confidence,
                std::hypot(
                    static_cast<double>(dx) * grid.cell,
                    static_cast<double>(dy) * grid.cell),
            });
        }
    }

    if (support.empty()) {
        return estimate;
    }

    auto weightedAverage = [](const std::vector<WeightedNeighbour>& values) {
        double sum_w = 0.0;
        double sum_z = 0.0;
        for (const auto& value : values) {
            sum_w += value.weight;
            sum_z += value.weight * value.z;
        }
        return std::make_pair(sum_w > 0.0 ? (sum_z / sum_w) : 0.0, sum_w);
    };

    const auto [provisional_z, provisional_w] = weightedAverage(support);
    (void)provisional_w;

    std::vector<WeightedNeighbour> consistent;
    consistent.reserve(support.size());
    const double allowed_extra =
        std::max(params.ground_support_band_down, params.ground_support_band_up);
    for (const auto& value : support) {
        const double allowed =
            std::max(0.02, params.propagation_max_slope * value.distance + allowed_extra);
        if (std::abs(value.z - provisional_z) <= allowed) {
            consistent.push_back(value);
        }
    }
    if (consistent.empty()) {
        consistent = support;
    }

    const auto [predicted_z, sum_w] = weightedAverage(consistent);
    estimate.valid = true;
    estimate.predicted_z = predicted_z;
    estimate.confidence = sum_w;
    estimate.contributors = static_cast<int>(consistent.size());
    return estimate;
}

static GridClassification classifyPropagatedGridCell(
    const PropagatedGridCell& cell,
    double predicted_z,
    double predicted_confidence,
    const ObstacleDetectionParams& params) {
    GridClassification out;
    out.ground_z = predicted_z;
    const double traversable_step_height = derivedTraversableStepHeight(params);
    const double overhang_clearance =
        (params.overhang_clearance_m > 0.0) ? params.overhang_clearance_m : params.obstacle_z_max;
    if (cell.point_count <= 0) {
        if (params.empty_cell_policy == GridEmptyCellPolicy::PropagateAcrossUnknown) {
            out.state = PropagatedCellState::PropagatedGround;
            out.confidence = std::max(0.10, predicted_confidence * 0.20);
            out.reason = "empty_cell_propagated";
        } else {
            out.state = PropagatedCellState::BlockedUnknown;
            out.confidence = 0.0;
            out.reason = "empty_cell_blocked";
        }
        return out;
    }

    const GroundSupportAnalysis support = analyzeGroundSupport(cell, predicted_z, params);
    out.support_point_count = support.support_point_count;
    out.support_base_z = support.support_base_z;
    out.support_top_z = support.support_top_z;
    out.support_span_z = support.support_span_z;
    out.first_non_ground_z = support.first_non_ground_z;
    out.clearance_above_ground = std::isfinite(support.clearance_above_ground)
        ? support.clearance_above_ground
        : 0.0;

    if (support.found_support) {
        if (support.support_span_z > traversable_step_height) {
            out.state = PropagatedCellState::MeasuredObstacle;
            out.confidence = 0.0;
            out.reason = "grounded_cluster_span";
            return out;
        }
        if (support.has_upper_cluster && support.clearance_above_ground <= overhang_clearance) {
            out.state = PropagatedCellState::MeasuredObstacle;
            out.confidence = 0.0;
            out.reason = "low_overhang";
            return out;
        }

        out.state = PropagatedCellState::PropagatedGround;
        out.ground_z = support.support_base_z;
        const double support_strength = static_cast<double>(support.support_point_count) /
                                        static_cast<double>(std::max(1, params.min_support_points));
        out.confidence =
            std::max(0.15, 0.50 * predicted_confidence + std::min(2.0, support_strength));
        out.reason = support.has_upper_cluster ? "ground_with_clear_overhang" : "ground_support";
        return out;
    }

    const double first_point_z = lowestValueSorted(cell.z_values);
    const double clearance_above_ground = std::max(0.0, first_point_z - predicted_z);
    out.first_non_ground_z = first_point_z;
    out.clearance_above_ground = clearance_above_ground;
    if (clearance_above_ground >= overhang_clearance) {
        out.state = PropagatedCellState::PropagatedGround;
        out.confidence = std::max(0.10, predicted_confidence * 0.35);
        out.traversable_overhang = true;
        out.reason = "overhang_only";
    } else if (cell.point_count < std::max(1, params.min_support_points)) {
        out.state = PropagatedCellState::BlockedUnknown;
        out.confidence = 0.0;
        out.reason = "insufficient_support_points";
    } else {
        out.state = PropagatedCellState::MeasuredObstacle;
        out.confidence = 0.0;
        out.reason = "no_ground_support";
    }
    return out;
}

struct GroundZGradientStats {
    double threshold = 0.0;
    double median_gradient = 0.0;
    double mad_gradient = 0.0;
    size_t edge_count = 0;
    size_t high_gradient_edges = 0;
};

static double quantileValueCopyDouble(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }
    q = std::clamp(q, 0.0, 1.0);
    const size_t idx = quantileIndex(values.size(), q);
    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(idx),
        values.end());
    return values[idx];
}

static double groundZLowQuantile(const ObstacleDetectionParams& params) {
    // Very-low quantile keeps the "minimum z" behaviour while reducing isolated low outliers.
    return std::clamp(std::min(params.raw_cell_low_quantile, 0.02), 0.0, 0.10);
}

static size_t initializeGroundZMap(
    PropagatedGrid* grid,
    const ObstacleDetectionParams& params) {
    if (!grid) {
        return 0;
    }
    const double q = groundZLowQuantile(params);
    size_t observed_cells = 0;
    for (auto& cell : grid->cells) {
        cell.ground_z_valid = false;
        cell.gradient_reachable = false;
        if (!cell.inside_scope || cell.z_values.empty()) {
            continue;
        }
        cell.ground_z = quantileValueSorted(cell.z_values, q);
        cell.ground_z_valid = true;
        cell.confidence = std::max(1.0, std::sqrt(static_cast<double>(cell.point_count)));
        cell.interpolation_contributors = 0;
        cell.support_point_count = cell.point_count;
        cell.support_base_z = cell.ground_z;
        cell.support_top_z = cell.high_z;
        cell.support_span_z = std::max(0.0, cell.support_top_z - cell.support_base_z);
        cell.reason = "ground_z_observed";
        observed_cells++;
    }
    return observed_cells;
}

static size_t fillEmptyGroundZCellsFromNeighbours(
    PropagatedGrid* grid,
    const ObstacleDetectionParams& params) {
    (void)params;
    if (!grid || grid->w <= 0 || grid->h <= 0) {
        return 0;
    }
    size_t filled_total = 0;
    const int max_passes = std::max(1, grid->w + grid->h);
    const std::array<std::pair<int, int>, 8> dirs = {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    }};

    for (int pass = 0; pass < max_passes; ++pass) {
        struct Update {
            size_t idx = 0;
            double z = 0.0;
            double confidence = 0.0;
            int contributors = 0;
        };
        std::vector<Update> updates;
        updates.reserve(grid->cells.size() / 8 + 1);

        for (int y = 0; y < grid->h; ++y) {
            for (int x = 0; x < grid->w; ++x) {
                auto& cell = grid->cells[propagatedGridIndex(*grid, x, y)];
                if (!cell.inside_scope || cell.ground_z_valid) {
                    continue;
                }

                double sum_z = 0.0;
                double sum_w = 0.0;
                int contributors = 0;
                for (const auto& [dx, dy] : dirs) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (!propagatedGridInBounds(*grid, nx, ny)) {
                        continue;
                    }
                    const auto& neighbour = grid->cells[propagatedGridIndex(*grid, nx, ny)];
                    if (!neighbour.inside_scope || !neighbour.ground_z_valid) {
                        continue;
                    }
                    const bool diagonal = (dx != 0 && dy != 0);
                    const double distance_weight = diagonal ? (1.0 / std::sqrt(2.0)) : 1.0;
                    const double w = distance_weight * std::max(0.10, neighbour.confidence);
                    sum_z += w * neighbour.ground_z;
                    sum_w += w;
                    contributors++;
                }
                if (contributors > 0 && sum_w > 0.0) {
                    updates.push_back(Update{
                        propagatedGridIndex(*grid, x, y),
                        sum_z / sum_w,
                        0.15 * sum_w,
                        contributors,
                    });
                }
            }
        }

        if (updates.empty()) {
            break;
        }
        for (const auto& update : updates) {
            auto& cell = grid->cells[update.idx];
            if (cell.ground_z_valid) {
                continue;
            }
            cell.ground_z = update.z;
            cell.ground_z_valid = true;
            cell.confidence = std::max(0.10, update.confidence);
            cell.interpolation_contributors = update.contributors;
            cell.support_base_z = cell.ground_z;
            cell.support_top_z = cell.high_z;
            cell.support_span_z = std::max(0.0, cell.support_top_z - cell.support_base_z);
            cell.reason = "ground_z_interpolated";
            filled_total++;
        }
    }
    return filled_total;
}

static GroundZGradientStats computeGroundZGradientStats(
    const PropagatedGrid& grid,
    const ObstacleDetectionParams& params) {
    GroundZGradientStats stats;
    std::vector<double> gradients;
    gradients.reserve(grid.cells.size() * 2);
    const std::array<std::pair<int, int>, 2> dirs = {{
        {1, 0}, {0, 1},
    }};

    for (int y = 0; y < grid.h; ++y) {
        for (int x = 0; x < grid.w; ++x) {
            const auto& cell = grid.cells[propagatedGridIndex(grid, x, y)];
            if (!cell.inside_scope || !cell.ground_z_valid) {
                continue;
            }
            for (const auto& [dx, dy] : dirs) {
                const int nx = x + dx;
                const int ny = y + dy;
                if (!propagatedGridInBounds(grid, nx, ny)) {
                    continue;
                }
                const auto& neighbour = grid.cells[propagatedGridIndex(grid, nx, ny)];
                if (!neighbour.inside_scope || !neighbour.ground_z_valid) {
                    continue;
                }
                const double gradient =
                    std::abs(neighbour.ground_z - cell.ground_z) / std::max(1e-6, grid.cell);
                gradients.push_back(gradient);
            }
        }
    }

    const double base_threshold = std::max(0.05, params.propagation_max_slope);
    stats.threshold = base_threshold;
    stats.edge_count = gradients.size();
    if (!gradients.empty()) {
        stats.median_gradient = quantileValueCopyDouble(gradients, 0.50);
        std::vector<double> abs_deviation;
        abs_deviation.reserve(gradients.size());
        for (double gradient : gradients) {
            abs_deviation.push_back(std::abs(gradient - stats.median_gradient));
        }
        stats.mad_gradient = quantileValueCopyDouble(abs_deviation, 0.50);
        const double robust_sigma = 1.4826 * stats.mad_gradient;
        const double adaptive_threshold = stats.median_gradient + 3.0 * robust_sigma;
        const double safety_cap = std::max(base_threshold, 1.5 * base_threshold);
        if (std::isfinite(adaptive_threshold) && adaptive_threshold > 0.0) {
            stats.threshold = std::clamp(adaptive_threshold, base_threshold, safety_cap);
        }
    }

    for (double gradient : gradients) {
        if (gradient > stats.threshold) {
            stats.high_gradient_edges++;
        }
    }
    return stats;
}

static bool groundZGradientEdgePassable(
    const PropagatedGrid& grid,
    int x0,
    int y0,
    int x1,
    int y1,
    double threshold) {
    if (!propagatedGridInBounds(grid, x0, y0) ||
        !propagatedGridInBounds(grid, x1, y1)) {
        return false;
    }
    const auto& a = grid.cells[propagatedGridIndex(grid, x0, y0)];
    const auto& b = grid.cells[propagatedGridIndex(grid, x1, y1)];
    if (!a.inside_scope || !b.inside_scope || !a.ground_z_valid || !b.ground_z_valid) {
        return false;
    }
    const double gradient = std::abs(a.ground_z - b.ground_z) / std::max(1e-6, grid.cell);
    return gradient <= threshold;
}

static double groundZGradientToNeighbour(
    const PropagatedGrid& grid,
    int x,
    int y,
    int dx,
    int dy,
    bool* valid_out = nullptr) {
    if (valid_out) {
        *valid_out = false;
    }
    const int nx = x + dx;
    const int ny = y + dy;
    if (!propagatedGridInBounds(grid, x, y) ||
        !propagatedGridInBounds(grid, nx, ny)) {
        return 0.0;
    }
    const auto& cell = grid.cells[propagatedGridIndex(grid, x, y)];
    const auto& neighbour = grid.cells[propagatedGridIndex(grid, nx, ny)];
    if (!cell.inside_scope || !neighbour.inside_scope ||
        !cell.ground_z_valid || !neighbour.ground_z_valid) {
        return 0.0;
    }
    if (valid_out) {
        *valid_out = true;
    }
    return std::abs(neighbour.ground_z - cell.ground_z) / std::max(1e-6, grid.cell);
}

static size_t seedGroundZGradientReachability(PropagatedGrid* grid) {
    if (!grid) {
        return 0;
    }
    size_t seeds = 0;
    for (auto& cell : grid->cells) {
        if (!cell.inside_scope || !cell.ground_z_valid || !cell.trail_covered) {
            continue;
        }
        cell.gradient_reachable = true;
        cell.state = PropagatedCellState::AnchorGround;
        cell.confidence = std::max(cell.confidence, 10.0);
        cell.incoming_gradient = 0.0;
        cell.incoming_gradient_valid = false;
        cell.incoming_from_x = -1;
        cell.incoming_from_y = -1;
        cell.reason = "ground_z_path_seed";
        seeds++;
    }

    if (seeds > 0) {
        return seeds;
    }

    size_t best_idx = 0;
    double best_z = std::numeric_limits<double>::infinity();
    bool found = false;
    for (size_t idx = 0; idx < grid->cells.size(); ++idx) {
        const auto& cell = grid->cells[idx];
        if (!cell.inside_scope || !cell.ground_z_valid) {
            continue;
        }
        if (!found || cell.ground_z < best_z) {
            best_idx = idx;
            best_z = cell.ground_z;
            found = true;
        }
    }
    if (found) {
        auto& cell = grid->cells[best_idx];
        cell.gradient_reachable = true;
        cell.state = PropagatedCellState::AnchorGround;
        cell.confidence = std::max(cell.confidence, 5.0);
        cell.incoming_gradient = 0.0;
        cell.incoming_gradient_valid = false;
        cell.incoming_from_x = -1;
        cell.incoming_from_y = -1;
        cell.reason = "ground_z_lowest_seed";
        return 1;
    }
    return 0;
}

static ObstacleDebugInfo makeGridCellDebugInfo(
    const PropagatedGrid& grid,
    int x,
    int y,
    double gradient_threshold,
    bool ground_cell) {
    const auto& cell = grid.cells[propagatedGridIndex(grid, x, y)];
    const Point2D center = propagatedGridCellCenter(grid, x, y);
    ObstacleDebugInfo debug;
    debug.enabled = true;
    debug.ground_cell = ground_cell;
    debug.cell_x = x;
    debug.cell_y = y;
    debug.center_x = center.x;
    debug.center_y = center.y;
    debug.ground_z_valid = cell.ground_z_valid;
    debug.z_est = cell.ground_z;
    debug.confidence = cell.confidence;
    debug.trail_covered = cell.trail_covered;
    debug.gradient_reachable = cell.gradient_reachable;
    debug.interpolation_contributors = cell.interpolation_contributors;
    debug.point_count = cell.point_count;
    debug.low_z = cell.low_z;
    debug.median_z = cell.median_z;
    debug.high_z = cell.high_z;
    debug.vertical_span = cell.vertical_span;
    debug.gradient_east = groundZGradientToNeighbour(
        grid, x, y, 1, 0, &debug.gradient_east_valid);
    debug.gradient_north = groundZGradientToNeighbour(
        grid, x, y, 0, 1, &debug.gradient_north_valid);
    if (debug.gradient_east_valid && debug.gradient_north_valid) {
        debug.gradient_max = std::max(debug.gradient_east, debug.gradient_north);
    } else if (debug.gradient_east_valid) {
        debug.gradient_max = debug.gradient_east;
    } else if (debug.gradient_north_valid) {
        debug.gradient_max = debug.gradient_north;
    } else {
        debug.gradient_max = 0.0;
    }
    debug.gradient_threshold = gradient_threshold;
    debug.high_gradient_east = debug.gradient_east_valid && debug.gradient_east > gradient_threshold;
    debug.high_gradient_north = debug.gradient_north_valid && debug.gradient_north > gradient_threshold;
    const std::array<std::pair<int, int>, 4> cardinal_dirs = {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    }};
    for (const auto& [dx, dy] : cardinal_dirs) {
        bool valid = false;
        const double gradient = groundZGradientToNeighbour(grid, x, y, dx, dy, &valid);
        if (!valid) {
            continue;
        }
        debug.valid_cardinal_edges++;
        if (!debug.gradient_cardinal_max_valid || gradient > debug.gradient_cardinal_max) {
            debug.gradient_cardinal_max = gradient;
            debug.gradient_cardinal_max_valid = true;
        }
        if (gradient <= gradient_threshold) {
            debug.passable_cardinal_edges++;
        } else {
            debug.blocked_cardinal_edges++;
        }
    }
    debug.incoming_gradient = cell.incoming_gradient;
    debug.incoming_gradient_valid = cell.incoming_gradient_valid;
    debug.incoming_from_cell_x = cell.incoming_from_x;
    debug.incoming_from_cell_y = cell.incoming_from_y;
    debug.support_point_count = cell.support_point_count;
    debug.support_base_z = cell.support_base_z;
    debug.support_top_z = cell.support_top_z;
    debug.support_span_z = cell.support_span_z;
    debug.first_non_ground_z = cell.first_non_ground_z;
    debug.clearance_above_ground = cell.clearance_above_ground;
    debug.reason = cell.reason;
    return debug;
}

static Obstacle2D gridCellRectObstacle(
    const PropagatedGrid& grid,
    int x,
    int y,
    ObstacleVisualType visual_type,
    double gradient_threshold,
    bool ground_cell) {
    const Point2D center = propagatedGridCellCenter(grid, x, y);
    const double half = 0.5 * grid.cell;
    Obstacle2D obstacle;
    obstacle.outer = {
        {center.x - half, center.y - half},
        {center.x + half, center.y - half},
        {center.x + half, center.y + half},
        {center.x - half, center.y + half},
    };
    obstacle.visual_type = visual_type;
    obstacle.debug_info = makeGridCellDebugInfo(grid, x, y, gradient_threshold, ground_cell);
    return obstacle;
}

static std::vector<Obstacle2D> buildGroundDebugCells(
    const PropagatedGrid& grid,
    double gradient_threshold,
    double min_contour_area_m2) {
    std::vector<Obstacle2D> cells;
    cells.reserve(grid.cells.size());
    const double cell_min_area = std::max(0.0, min_contour_area_m2);
    for (int y = 0; y < grid.h; ++y) {
        for (int x = 0; x < grid.w; ++x) {
            const auto& cell = grid.cells[propagatedGridIndex(grid, x, y)];
            if (!cell.inside_scope || !isGroundCellState(cell.state)) {
                continue;
            }
            Obstacle2D debug_cell = gridCellRectObstacle(
                grid, x, y, ObstacleVisualType::Ground, gradient_threshold, true);
            if (polygonArea(debug_cell.outer) >= cell_min_area) {
                cells.push_back(std::move(debug_cell));
            }
        }
    }
    return cells;
}

static void classifyGroundZGradientGrid(
    PropagatedGrid* grid,
    const GroundZGradientStats& gradient_stats,
    const ObstacleDetectionParams& params,
    ObstacleDetectionStats* stats_out) {
    (void)params;
    if (!grid || grid->w <= 0 || grid->h <= 0) {
        return;
    }

    std::deque<std::pair<int, int>> queue;
    size_t anchor_cells = 0;
    for (int y = 0; y < grid->h; ++y) {
        for (int x = 0; x < grid->w; ++x) {
            auto& cell = grid->cells[propagatedGridIndex(*grid, x, y)];
            if (!cell.gradient_reachable) {
                continue;
            }
            queue.emplace_back(x, y);
            anchor_cells++;
        }
    }

    const std::array<std::pair<int, int>, 4> dirs = {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    }};
    size_t reachable_ground_cells = 0;
    while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop_front();
        auto& cell = grid->cells[propagatedGridIndex(*grid, x, y)];
        if (cell.state != PropagatedCellState::AnchorGround) {
            cell.state = PropagatedCellState::PropagatedGround;
            cell.reason = cell.point_count > 0 ? "ground_z_reachable" : "ground_z_reachable_filled";
        }
        reachable_ground_cells++;

        for (const auto& [dx, dy] : dirs) {
            const int nx = x + dx;
            const int ny = y + dy;
            bool edge_valid = false;
            const double edge_gradient =
                groundZGradientToNeighbour(*grid, x, y, dx, dy, &edge_valid);
            if (!edge_valid || edge_gradient > gradient_stats.threshold) {
                continue;
            }
            auto& neighbour = grid->cells[propagatedGridIndex(*grid, nx, ny)];
            if (neighbour.gradient_reachable) {
                continue;
            }
            neighbour.gradient_reachable = true;
            neighbour.state = PropagatedCellState::PropagatedGround;
            neighbour.confidence = std::max(0.25, 0.75 * cell.confidence);
            neighbour.incoming_gradient = edge_gradient;
            neighbour.incoming_gradient_valid = true;
            neighbour.incoming_from_x = x;
            neighbour.incoming_from_y = y;
            queue.emplace_back(nx, ny);
        }
    }

    size_t blocked_unknown_cells = 0;
    size_t measured_obstacle_cells = 0;
    size_t ground_points_band = 0;
    for (auto& cell : grid->cells) {
        if (!cell.inside_scope) {
            cell.state = PropagatedCellState::Outside;
            continue;
        }
        if (isGroundCellState(cell.state)) {
            ground_points_band += static_cast<size_t>(std::max(0, cell.point_count));
            continue;
        }
        if (!cell.ground_z_valid || cell.point_count <= 0) {
            cell.state = PropagatedCellState::BlockedUnknown;
            cell.confidence = 0.0;
            cell.reason = cell.ground_z_valid ? "ground_z_empty_unreachable" : "ground_z_missing";
            blocked_unknown_cells++;
            continue;
        }

        cell.state = PropagatedCellState::MeasuredObstacle;
        cell.confidence = 0.0;
        cell.support_point_count = cell.point_count;
        cell.support_base_z = cell.ground_z;
        cell.support_top_z = cell.high_z;
        cell.support_span_z = std::max(0.0, cell.support_top_z - cell.support_base_z);
        cell.first_non_ground_z = cell.low_z;
        cell.clearance_above_ground = std::max(0.0, cell.low_z - cell.ground_z);
        cell.reason = "ground_z_unreachable_island";
        measured_obstacle_cells++;
    }

    if (stats_out) {
        stats_out->anchor_cells = anchor_cells;
        stats_out->propagated_ground_cells = reachable_ground_cells;
        stats_out->blocked_unknown_cells = blocked_unknown_cells;
        stats_out->measured_obstacle_cells = measured_obstacle_cells;
        stats_out->ground_points_band = ground_points_band;
        stats_out->high_gradient_edges = gradient_stats.high_gradient_edges;
        stats_out->gradient_threshold = gradient_stats.threshold;
        stats_out->raw_obstacle_candidates = measured_obstacle_cells + blocked_unknown_cells;
        stats_out->obstacle_points_after_outlier = stats_out->raw_obstacle_candidates;
    }
}

// --------------------- Statistical outlier removal (Python-like) ------------

static PointCloudPtr removeStatisticalOutliersMeanDist(
    const PointCloudPtr& cloud,
    int k,
    double std_ratio) {
    if (!cloud) return PointCloudPtr(new PointCloud);
    const size_t n = cloud->size();
    if (n == 0) return PointCloudPtr(new PointCloud);
    if (k < 1) k = 1;
    if (n < static_cast<size_t>(k + 1)) {
        return cloud;  // match python: too few points => keep all
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    const int K = k + 1;  // include self
    std::vector<int> idx(K);
    std::vector<float> dist2(K);

    std::vector<double> mean_dists(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const pcl::PointXYZ& p = cloud->points[i];
        const int found = tree.nearestKSearch(p, K, idx, dist2);
        if (found <= 1) {
            mean_dists[i] = 0.0;
            continue;
        }
        double sum = 0.0;
        int cnt = 0;
        for (int j = 1; j < found; ++j) {  // skip self (dist=0)
            sum += std::sqrt(std::max(0.0f, dist2[j]));
            cnt++;
        }
        mean_dists[i] = (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
    }

    double mu = 0.0;
    for (double v : mean_dists) mu += v;
    mu /= std::max<size_t>(1, n);

    double var = 0.0;
    for (double v : mean_dists) {
        const double d = v - mu;
        var += d * d;
    }
    var /= std::max<size_t>(1, n);
    const double sigma = std::sqrt(std::max(0.0, var));
    const double threshold = mu + std_ratio * sigma;

    PointCloudPtr out(new PointCloud);
    out->reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (mean_dists[i] <= threshold) {
            out->push_back(cloud->points[i]);
        }
    }
    return out;
}

// --------------------------- DBSCAN (2D) ------------------------------------

static std::vector<int> dbscan2D(const std::vector<Point2D>& pts, double eps, int min_samples) {
    const int n = static_cast<int>(pts.size());
    std::vector<int> labels(n, -1);
    if (n == 0) return labels;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xy->reserve(pts.size());
    for (const auto& p : pts) {
        cloud_xy->push_back(pcl::PointXYZ(static_cast<float>(p.x), static_cast<float>(p.y), 0.0f));
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud_xy);

    std::vector<std::vector<int>> neighbours(n);
    neighbours.reserve(n);
    const float eps_f = static_cast<float>(eps);
    for (int i = 0; i < n; ++i) {
        std::vector<int> idx;
        std::vector<float> dist2;
        tree.radiusSearch(cloud_xy->points[i], eps_f, idx, dist2);
        neighbours[i] = std::move(idx);
    }

    std::vector<uint8_t> core(n, 0);
    for (int i = 0; i < n; ++i) {
        if (static_cast<int>(neighbours[i].size()) >= min_samples) {
            core[i] = 1;
        }
    }

    int cluster_id = 0;
    for (int seed = 0; seed < n; ++seed) {
        if (labels[seed] != -1 || !core[seed]) continue;
        std::deque<int> q;
        q.push_back(seed);
        labels[seed] = cluster_id;
        while (!q.empty()) {
            int curr = q.front();
            q.pop_front();
            for (int nb : neighbours[curr]) {
                if (labels[nb] != -1) continue;
                labels[nb] = cluster_id;
                if (core[nb]) {
                    q.push_back(nb);
                }
            }
        }
        cluster_id++;
    }

    return labels;
}

// --------------------- Occupancy grid & contour rings -----------------------

struct OccGrid {
    int w = 0;
    int h = 0;
    double xmin = 0.0;
    double ymin = 0.0;
    double cell = 0.09;
    std::vector<uint8_t> occ;  // row-major (y then x), 1=occupied
};

static inline bool occAt(const OccGrid& g, int x, int y) {
    if (x < 0 || y < 0 || x >= g.w || y >= g.h) return false;
    return g.occ[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)] != 0;
}

struct ScalarGrid {
    int w = 0;
    int h = 0;
    double xmin = 0.0;
    double ymin = 0.0;
    double cell = 0.09;
    std::vector<double> values;  // row-major signed distance, +inside / -outside
};

static inline double scalarAt(const ScalarGrid& g, int x, int y) {
    if (x < 0 || y < 0 || x >= g.w || y >= g.h) {
        return -0.5 * g.cell;
    }
    return g.values[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)];
}

using ShapeContours = std::vector<std::pair<Polygon2D, std::vector<Polygon2D>>>;

static std::vector<double> squaredDistanceToFeature(const OccGrid& g, bool feature_occupied) {
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> row_sq(static_cast<size_t>(g.w) * static_cast<size_t>(g.h), inf);
    std::vector<double> out(static_cast<size_t>(g.w) * static_cast<size_t>(g.h), inf);

    for (int y = 0; y < g.h; ++y) {
        int last = -1;
        for (int x = 0; x < g.w; ++x) {
            if ((occAt(g, x, y) != 0) == feature_occupied) {
                last = x;
            }
            if (last >= 0) {
                const double dx = static_cast<double>(x - last);
                row_sq[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)] = dx * dx;
            }
        }

        last = -1;
        for (int x = g.w - 1; x >= 0; --x) {
            if ((occAt(g, x, y) != 0) == feature_occupied) {
                last = x;
            }
            if (last >= 0) {
                const double dx = static_cast<double>(last - x);
                double& v = row_sq[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)];
                v = std::min(v, dx * dx);
            }
        }
    }

    for (int x = 0; x < g.w; ++x) {
        std::vector<std::pair<int, double>> candidates;
        candidates.reserve(static_cast<size_t>(g.h));
        for (int y = 0; y < g.h; ++y) {
            const double base = row_sq[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)];
            if (std::isfinite(base)) {
                candidates.emplace_back(y, base);
            }
        }
        if (candidates.empty()) {
            continue;
        }

        for (int y = 0; y < g.h; ++y) {
            double best = inf;
            for (const auto& [cy, base] : candidates) {
                const double dy = static_cast<double>(y - cy);
                best = std::min(best, base + dy * dy);
            }
            out[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)] = best;
        }
    }

    return out;
}

static ScalarGrid signedDistanceFieldFromOcc(const OccGrid& occ) {
    ScalarGrid g;
    if (occ.w <= 0 || occ.h <= 0 || occ.occ.empty()) {
        return g;
    }

    g.w = occ.w;
    g.h = occ.h;
    g.xmin = occ.xmin;
    g.ymin = occ.ymin;
    g.cell = occ.cell;
    g.values.assign(static_cast<size_t>(g.w) * static_cast<size_t>(g.h), 0.0);

    const std::vector<double> dist_occ_sq = squaredDistanceToFeature(occ, true);
    const std::vector<double> dist_empty_sq = squaredDistanceToFeature(occ, false);
    for (size_t i = 0; i < g.values.size(); ++i) {
        const double d_occ = std::isfinite(dist_occ_sq[i]) ? std::sqrt(std::max(0.0, dist_occ_sq[i])) : 0.0;
        const double d_empty = std::isfinite(dist_empty_sq[i]) ? std::sqrt(std::max(0.0, dist_empty_sq[i])) : 0.0;
        // A center-based signed distance keeps the same topology as the binary mask,
        // while shifting iso-crossings away from fixed midpoints on long oblique runs.
        g.values[i] = 0.5 * (d_empty - d_occ) * occ.cell;
    }
    return g;
}

static OccGrid occupancyFromPoints(const std::vector<Point2D>& pts, double cell, double padding) {
    OccGrid g;
    g.cell = cell;
    if (pts.empty() || cell <= 0) {
        return g;
    }
    double minx = pts[0].x, maxx = pts[0].x;
    double miny = pts[0].y, maxy = pts[0].y;
    for (const auto& p : pts) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }
    g.xmin = minx - padding;
    double xmax = maxx + padding;
    g.ymin = miny - padding;
    double ymax = maxy + padding;

    g.w = std::max(1, static_cast<int>(std::ceil((xmax - g.xmin) / cell)));
    g.h = std::max(1, static_cast<int>(std::ceil((ymax - g.ymin) / cell)));
    g.occ.assign(static_cast<size_t>(g.w) * static_cast<size_t>(g.h), 0);

    for (const auto& p : pts) {
        int ix = static_cast<int>(std::floor((p.x - g.xmin) / cell));
        int iy = static_cast<int>(std::floor((p.y - g.ymin) / cell));
        ix = std::max(0, std::min(g.w - 1, ix));
        iy = std::max(0, std::min(g.h - 1, iy));
        g.occ[static_cast<size_t>(iy) * static_cast<size_t>(g.w) + static_cast<size_t>(ix)] = 1;
    }
    return g;
}

static OccGrid inflateOccupancy(const OccGrid& in, double radius_m) {
    if (in.w <= 0 || in.h <= 0 || in.occ.empty()) return in;
    if (radius_m <= 1e-9) return in;

    int r = static_cast<int>(std::ceil(radius_m / in.cell));
    if (r <= 0) return in;

    std::vector<std::pair<int, int>> offsets;
    offsets.reserve(static_cast<size_t>((2 * r + 1) * (2 * r + 1)));
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy <= r * r) {
                offsets.emplace_back(dx, dy);
            }
        }
    }

    OccGrid out = in;
    std::fill(out.occ.begin(), out.occ.end(), 0);

    for (int y = 0; y < in.h; ++y) {
        for (int x = 0; x < in.w; ++x) {
            if (!occAt(in, x, y)) continue;
            for (const auto& [dx, dy] : offsets) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= in.w || ny >= in.h) continue;
                out.occ[static_cast<size_t>(ny) * static_cast<size_t>(in.w) + static_cast<size_t>(nx)] = 1;
            }
        }
    }
    return out;
}

static std::vector<std::pair<int, int>> circleOffsets(int r) {
    std::vector<std::pair<int, int>> offsets;
    if (r <= 0) return offsets;
    offsets.reserve(static_cast<size_t>((2 * r + 1) * (2 * r + 1)));
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy <= r * r) {
                offsets.emplace_back(dx, dy);
            }
        }
    }
    return offsets;
}

static OccGrid erodeOccupancy(const OccGrid& in, double radius_m) {
    if (in.w <= 0 || in.h <= 0 || in.occ.empty()) return in;
    if (radius_m <= 1e-9) return in;

    int r = static_cast<int>(std::ceil(radius_m / in.cell));
    if (r <= 0) return in;

    const auto offsets = circleOffsets(r);
    if (offsets.empty()) return in;

    OccGrid out = in;
    std::fill(out.occ.begin(), out.occ.end(), 0);

    for (int y = 0; y < in.h; ++y) {
        for (int x = 0; x < in.w; ++x) {
            bool keep = true;
            for (const auto& [dx, dy] : offsets) {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= in.w || ny >= in.h) {
                    keep = false;
                    break;
                }
                if (!occAt(in, nx, ny)) {
                    keep = false;
                    break;
                }
            }
            if (keep) {
                out.occ[static_cast<size_t>(y) * static_cast<size_t>(in.w) + static_cast<size_t>(x)] = 1;
            }
        }
    }
    return out;
}

static OccGrid closeOccupancyConservative(const OccGrid& in, double radius_m) {
    // Morphological closing = dilation then erosion (conservative / extensive in infinite grid).
    if (radius_m <= 1e-9) return in;
    OccGrid dil = inflateOccupancy(in, radius_m);
    OccGrid clo = erodeOccupancy(dil, radius_m);
    return clo;
}

static OccGrid maxpoolOccupancy(const OccGrid& in, int factor) {
    if (in.w <= 0 || in.h <= 0 || in.occ.empty()) return in;
    if (factor <= 1) return in;

    const int pad_w = (factor - (in.w % factor)) % factor;
    const int pad_h = (factor - (in.h % factor)) % factor;
    const int w2 = in.w + pad_w;
    const int h2 = in.h + pad_h;

    std::vector<uint8_t> padded(static_cast<size_t>(w2) * static_cast<size_t>(h2), 0);
    for (int y = 0; y < in.h; ++y) {
        for (int x = 0; x < in.w; ++x) {
            padded[static_cast<size_t>(y) * static_cast<size_t>(w2) + static_cast<size_t>(x)] =
                in.occ[static_cast<size_t>(y) * static_cast<size_t>(in.w) + static_cast<size_t>(x)];
        }
    }

    const int out_w = w2 / factor;
    const int out_h = h2 / factor;
    OccGrid out;
    out.xmin = in.xmin;
    out.ymin = in.ymin;
    out.cell = in.cell * static_cast<double>(factor);
    out.w = out_w;
    out.h = out_h;
    out.occ.assign(static_cast<size_t>(out_w) * static_cast<size_t>(out_h), 0);

    for (int by = 0; by < out_h; ++by) {
        for (int bx = 0; bx < out_w; ++bx) {
            uint8_t mx = 0;
            const int y0 = by * factor;
            const int x0 = bx * factor;
            for (int dy = 0; dy < factor && mx == 0; ++dy) {
                for (int dx = 0; dx < factor; ++dx) {
                    const int ix = x0 + dx;
                    const int iy = y0 + dy;
                    mx = std::max<uint8_t>(mx, padded[static_cast<size_t>(iy) * static_cast<size_t>(w2) + static_cast<size_t>(ix)]);
                    if (mx) break;
                }
            }
            out.occ[static_cast<size_t>(by) * static_cast<size_t>(out_w) + static_cast<size_t>(bx)] = mx;
        }
    }
    return out;
}

// Forward declarations (used by helper routines below)
static std::vector<Polygon2D> extractContourRingsFromOcc(const OccGrid& g);
static std::vector<Polygon2D> extractContourRingsFromSdf(const ScalarGrid& g);
static ShapeContours groupRingsIntoShapes(
    std::vector<Polygon2D> rings,
    double min_area_m2);
static ShapeContours extractShapesFromOcc(const OccGrid& g, double min_area_m2);

static ShapeContours polygonizeClusterGrid(
    const std::vector<Point2D>& pts2d,
    double grid_cell_m,
    double contour_cell_m,
    double inflate_radius_m,
    double smooth_radius_m,
    double min_contour_area_m2) {
    if (pts2d.empty() || grid_cell_m <= 0.0) {
        return {};
    }

    // Match Python: padding=max(0.20, 2*inflate_radius)
    const double padding = std::max(0.20, 2.0 * std::max(0.0, inflate_radius_m));
    OccGrid occ = occupancyFromPoints(pts2d, grid_cell_m, padding);
    occ = inflateOccupancy(occ, std::max(0.0, inflate_radius_m));
    occ = closeOccupancyConservative(occ, std::max(0.0, smooth_radius_m));

    // Optional coarser contour grid (conservative max-pooling)
    if (contour_cell_m > 0.0 && contour_cell_m > grid_cell_m + 1e-12) {
        const int factor = std::max(1, static_cast<int>(std::round(contour_cell_m / grid_cell_m)));
        if (factor > 1) {
            occ = maxpoolOccupancy(occ, factor);
        }
    }

    return extractShapesFromOcc(occ, min_contour_area_m2);
}

static Polygon2D rectFromBbox(const std::vector<Point2D>& pts2d, double min_size_m, double margin_m) {
    if (pts2d.empty()) {
        const double half = 0.5 * min_size_m;
        return Polygon2D{
            { -half, -half },
            {  half, -half },
            {  half,  half },
            { -half,  half },
        };
    }
    double xmin = pts2d.front().x;
    double xmax = pts2d.front().x;
    double ymin = pts2d.front().y;
    double ymax = pts2d.front().y;
    for (const auto& p : pts2d) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    const double cx = 0.5 * (xmin + xmax);
    const double cy = 0.5 * (ymin + ymax);
    const double w = std::max(min_size_m, (xmax - xmin)) + 2.0 * margin_m;
    const double h = std::max(min_size_m, (ymax - ymin)) + 2.0 * margin_m;
    const double hw = 0.5 * w;
    const double hh = 0.5 * h;
    return Polygon2D{
        { cx - hw, cy - hh },
        { cx + hw, cy - hh },
        { cx + hw, cy + hh },
        { cx - hw, cy + hh },
    };
}

static bool isMicroCluster(const std::vector<Point2D>& pts2d, const ObstacleDetectionParams& params) {
    if (static_cast<int>(pts2d.size()) < params.micro_min_pts) {
        return false;
    }
    double xmin = pts2d.front().x;
    double xmax = pts2d.front().x;
    double ymin = pts2d.front().y;
    double ymax = pts2d.front().y;
    for (const auto& p : pts2d) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    const double span_x = xmax - xmin;
    const double span_y = ymax - ymin;
    if (std::max(span_x, span_y) > params.micro_max_span_m) {
        return false;
    }
    const double area = std::max(span_x, 1e-6) * std::max(span_y, 1e-6);
    const double density = static_cast<double>(pts2d.size()) / area;
    return density >= params.micro_min_density_pts_per_m2;
}

static bool pointInObstacleShape(const Point2D& p, const Obstacle2D& shape) {
    if (!pointInPolyRayCast(p, shape.outer)) {
        return false;
    }
    for (const auto& h : shape.holes) {
        if (pointInPolyRayCast(p, h)) {
            return false;
        }
    }
    return true;
}

static OccGrid rasterizeShapeToOccupancy(
    const Obstacle2D& shape,
    double cell_size,
    double padding) {
    OccGrid g;
    if (shape.outer.size() < 3 || cell_size <= 0.0) {
        return g;
    }
    g.cell = cell_size;

    double xmin = shape.outer.front().x;
    double xmax = shape.outer.front().x;
    double ymin = shape.outer.front().y;
    double ymax = shape.outer.front().y;
    for (const auto& p : shape.outer) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    g.xmin = xmin - padding;
    g.ymin = ymin - padding;
    const double xmax2 = xmax + padding;
    const double ymax2 = ymax + padding;

    g.w = std::max(1, static_cast<int>(std::ceil((xmax2 - g.xmin) / cell_size)));
    g.h = std::max(1, static_cast<int>(std::ceil((ymax2 - g.ymin) / cell_size)));
    g.occ.assign(static_cast<size_t>(g.w) * static_cast<size_t>(g.h), 0);

    for (int y = 0; y < g.h; ++y) {
        const double wy = g.ymin + (static_cast<double>(y) + 0.5) * cell_size;
        for (int x = 0; x < g.w; ++x) {
            const double wx = g.xmin + (static_cast<double>(x) + 0.5) * cell_size;
            if (pointInObstacleShape(Point2D(wx, wy), shape)) {
                g.occ[static_cast<size_t>(y) * static_cast<size_t>(g.w) + static_cast<size_t>(x)] = 1;
            }
        }
    }
    return g;
}

static std::vector<Obstacle2D> smoothShapesRollingDiskGrid(
    const std::vector<Obstacle2D>& shapes,
    double radius_m,
    double cell_size,
    double contour_cell_m,
    double min_contour_area_m2,
    bool preserve_holes,
    double preserve_holes_min_area_m2) {
    if (radius_m <= 1e-9 || shapes.empty()) {
        return shapes;
    }

    std::vector<Obstacle2D> out;
    out.reserve(shapes.size());

    for (const auto& sh : shapes) {
        const double padding = std::max(0.20, 2.0 * radius_m);
        OccGrid occ = rasterizeShapeToOccupancy(sh, cell_size, padding);
        if (occ.occ.empty()) {
            out.push_back(sh);
            continue;
        }

        OccGrid occ2 = closeOccupancyConservative(occ, radius_m);

        // Preserve large holes: prevent smoothing from filling navigable cavities.
        if (preserve_holes && !sh.holes.empty() && preserve_holes_min_area_m2 > 0.0) {
            std::vector<const Polygon2D*> holes_keep;
            holes_keep.reserve(sh.holes.size());
            for (const auto& h : sh.holes) {
                if (polygonArea(h) >= preserve_holes_min_area_m2) {
                    holes_keep.push_back(&h);
                }
            }
            if (!holes_keep.empty()) {
                for (int y = 0; y < occ2.h; ++y) {
                    const double wy = occ2.ymin + (static_cast<double>(y) + 0.5) * occ2.cell;
                    for (int x = 0; x < occ2.w; ++x) {
                        if (!occAt(occ2, x, y)) {
                            continue;
                        }
                        const double wx = occ2.xmin + (static_cast<double>(x) + 0.5) * occ2.cell;
                        const Point2D p(wx, wy);
                        bool in_preserved_hole = false;
                        for (const auto* hptr : holes_keep) {
                            if (pointInPolyRayCast(p, *hptr)) {
                                in_preserved_hole = true;
                                break;
                            }
                        }
                        if (in_preserved_hole) {
                            occ2.occ[static_cast<size_t>(y) * static_cast<size_t>(occ2.w) + static_cast<size_t>(x)] = 0;
                        }
                    }
                }
            }
        }

        // Optional coarser contour grid (conservative max-pooling)
        if (contour_cell_m > 0.0 && contour_cell_m > cell_size + 1e-12) {
            const int factor = std::max(1, static_cast<int>(std::round(contour_cell_m / cell_size)));
            if (factor > 1) {
                occ2 = maxpoolOccupancy(occ2, factor);
            }
        }

        auto grouped = extractShapesFromOcc(occ2, min_contour_area_m2);
        if (grouped.empty()) {
            out.push_back(sh);
            continue;
        }

        for (auto& gsh : grouped) {
            Obstacle2D obs;
            obs.outer = std::move(gsh.first);
            obs.holes = std::move(gsh.second);
            out.push_back(std::move(obs));
        }
    }

    return out;
}

template <typename SampleFn>
static std::vector<Polygon2D> extractContourRingsGeneric(
    int w,
    int h,
    double xmin,
    double ymin,
    double cell,
    double iso,
    const SampleFn& sampleValue) {
    struct Segment {
        Point2D a;
        Point2D b;
    };
    struct Key {
        int64_t x = 0;
        int64_t y = 0;
        bool operator==(const Key& o) const { return x == o.x && y == o.y; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            return static_cast<size_t>((static_cast<uint64_t>(k.x) * 1315423911ULL) ^
                                       (static_cast<uint64_t>(k.y) + 0x9e3779b97f4a7c15ULL));
        }
    };

    std::vector<Polygon2D> rings;
    if (w <= 0 || h <= 0) {
        return rings;
    }

    const double snap = std::max(1e-9, cell * 1e-6);
    const double close_tol = snap * 8.0;

    auto toKey = [&](const Point2D& p) -> Key {
        return Key{
            static_cast<int64_t>(std::llround(p.x / snap)),
            static_cast<int64_t>(std::llround(p.y / snap))
        };
    };

    auto interp = [&](const Point2D& p0, const Point2D& p1, double v0, double v1) -> Point2D {
        double t = 0.5;
        const double dv = v1 - v0;
        if (std::abs(dv) > 1e-12) {
            t = (iso - v0) / dv;
        }
        t = std::clamp(t, 0.0, 1.0);
        return Point2D(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t);
    };

    auto samplePos = [](int cx, int cy) -> Point2D {
        return Point2D(static_cast<double>(cx) + 0.5, static_cast<double>(cy) + 0.5);
    };

    std::vector<Segment> segments;
    segments.reserve(static_cast<size_t>(w) * static_cast<size_t>(h) * 2);

    // Marching squares over 2x2 blocks of occupancy-cell samples.
    // This preserves tiny occupied islands better than averaged vertex fields.
    for (int y = -1; y < h; ++y) {
        for (int x = -1; x < w; ++x) {
            const double v0 = sampleValue(x, y);         // bottom-left
            const double v1 = sampleValue(x + 1, y);     // bottom-right
            const double v2 = sampleValue(x + 1, y + 1); // top-right
            const double v3 = sampleValue(x, y + 1);     // top-left

            const int c0 = (v0 > iso) ? 1 : 0;
            const int c1 = (v1 > iso) ? 1 : 0;
            const int c2 = (v2 > iso) ? 1 : 0;
            const int c3 = (v3 > iso) ? 1 : 0;
            const int idx = c0 | (c1 << 1) | (c2 << 2) | (c3 << 3);
            if (idx == 0 || idx == 15) {
                continue;
            }

            const Point2D p0 = samplePos(x, y);
            const Point2D p1 = samplePos(x + 1, y);
            const Point2D p2 = samplePos(x + 1, y + 1);
            const Point2D p3 = samplePos(x, y + 1);

            const Point2D e0 = interp(p0, p1, v0, v1); // bottom
            const Point2D e1 = interp(p1, p2, v1, v2); // right
            const Point2D e2 = interp(p2, p3, v2, v3); // top
            const Point2D e3 = interp(p3, p0, v3, v0); // left

            auto addSeg = [&](const Point2D& a, const Point2D& b) {
                if (std::hypot(a.x - b.x, a.y - b.y) > 1e-12) {
                    segments.push_back(Segment{a, b});
                }
            };

            switch (idx) {
                case 1:  addSeg(e3, e0); break;
                case 2:  addSeg(e0, e1); break;
                case 3:  addSeg(e3, e1); break;
                case 4:  addSeg(e1, e2); break;
                case 5: {
                    const double center = 0.25 * (v0 + v1 + v2 + v3);
                    if (center > iso) { addSeg(e3, e0); addSeg(e2, e1); }
                    else              { addSeg(e3, e2); addSeg(e0, e1); }
                    break;
                }
                case 6:  addSeg(e0, e2); break;
                case 7:  addSeg(e3, e2); break;
                case 8:  addSeg(e2, e3); break;
                case 9:  addSeg(e0, e2); break;
                case 10: {
                    const double center = 0.25 * (v0 + v1 + v2 + v3);
                    if (center > iso) { addSeg(e0, e3); addSeg(e1, e2); }
                    else              { addSeg(e0, e1); addSeg(e2, e3); }
                    break;
                }
                case 11: addSeg(e1, e2); break;
                case 12: addSeg(e1, e3); break;
                case 13: addSeg(e0, e1); break;
                case 14: addSeg(e3, e0); break;
                default: break;
            }
        }
    }

    if (segments.empty()) {
        return rings;
    }

    std::unordered_map<Key, std::vector<std::pair<int, int>>, KeyHash> endpoint_map;
    endpoint_map.reserve(segments.size() * 2);
    for (int i = 0; i < static_cast<int>(segments.size()); ++i) {
        endpoint_map[toKey(segments[static_cast<size_t>(i)].a)].push_back({i, 0});
        endpoint_map[toKey(segments[static_cast<size_t>(i)].b)].push_back({i, 1});
    }

    std::vector<uint8_t> used(segments.size(), 0);
    auto toWorld = [&](const Point2D& p) -> Point2D {
        return Point2D(xmin + p.x * cell, ymin + p.y * cell);
    };

    for (int si = 0; si < static_cast<int>(segments.size()); ++si) {
        if (used[static_cast<size_t>(si)]) {
            continue;
        }
        used[static_cast<size_t>(si)] = 1;

        Polygon2D ring;
        ring.reserve(128);
        Point2D start = segments[static_cast<size_t>(si)].a;
        Point2D curr = segments[static_cast<size_t>(si)].b;
        ring.push_back(toWorld(start));
        ring.push_back(toWorld(curr));

        int guard = 0;
        const int guard_max = static_cast<int>(segments.size()) * 2 + 16;
        bool closed = false;
        while (++guard <= guard_max) {
            if (std::hypot(curr.x - start.x, curr.y - start.y) <= close_tol) {
                closed = true;
                break;
            }
            auto it = endpoint_map.find(toKey(curr));
            if (it == endpoint_map.end()) {
                break;
            }

            int next_seg = -1;
            int next_end = -1;
            for (const auto& ent : it->second) {
                const int seg_idx = ent.first;
                const int end_idx = ent.second;
                if (used[static_cast<size_t>(seg_idx)]) continue;
                next_seg = seg_idx;
                next_end = end_idx;
                break;
            }

            if (next_seg < 0) {
                break;
            }
            used[static_cast<size_t>(next_seg)] = 1;
            const Segment& s = segments[static_cast<size_t>(next_seg)];
            const Point2D nxt = (next_end == 0) ? s.b : s.a;
            curr = nxt;
            ring.push_back(toWorld(curr));
        }

        if (!closed && ring.size() >= 4) {
            // Defensive close for tiny numerical cracks.
            const Point2D& first = ring.front();
            const Point2D& last = ring.back();
            if (std::hypot(last.x - first.x, last.y - first.y) <= close_tol * cell) {
                ring.back() = first;  // close cleanly for downstream cleaning.
                closed = true;
            }
        }

        if (closed && ring.size() >= 4) {
            rings.push_back(std::move(ring));
        }
    }

    return rings;
}

static std::vector<Polygon2D> extractContourRingsFromOcc(const OccGrid& g) {
    if (g.w <= 0 || g.h <= 0 || g.occ.empty()) {
        return {};
    }
    auto sample = [&](int cx, int cy) -> double {
        // Marching-squares samples live on cell centers; outside grid is empty.
        return occAt(g, cx, cy) ? 1.0 : 0.0;
    };
    return extractContourRingsGeneric(g.w, g.h, g.xmin, g.ymin, g.cell, 0.5, sample);
}

static std::vector<Polygon2D> extractContourRingsFromSdf(const ScalarGrid& g) {
    if (g.w <= 0 || g.h <= 0 || g.values.empty()) {
        return {};
    }
    auto sample = [&](int cx, int cy) -> double {
        return scalarAt(g, cx, cy);
    };
    return extractContourRingsGeneric(g.w, g.h, g.xmin, g.ymin, g.cell, 0.0, sample);
}

static void removeConsecutiveDuplicates(Polygon2D& ring) {
    if (ring.empty()) return;
    Polygon2D out;
    out.reserve(ring.size());
    out.push_back(ring.front());
    for (size_t i = 1; i < ring.size(); ++i) {
        const auto& prev = out.back();
        const auto& curr = ring[i];
        if (std::hypot(curr.x - prev.x, curr.y - prev.y) > 1e-9) {
            out.push_back(curr);
        }
    }
    ring.swap(out);
}

static void removeCollinear(Polygon2D& ring) {
    if (ring.size() < 3) return;

    auto cross = [](const Point2D& a, const Point2D& b, const Point2D& c) -> double {
        // cross((b-a),(c-b))
        double abx = b.x - a.x;
        double aby = b.y - a.y;
        double bcx = c.x - b.x;
        double bcy = c.y - b.y;
        return abx * bcy - aby * bcx;
    };

    bool changed = true;
    while (changed && ring.size() >= 3) {
        changed = false;
        Polygon2D out;
        out.reserve(ring.size());
        for (size_t i = 0; i < ring.size(); ++i) {
            const Point2D& prev = ring[(i + ring.size() - 1) % ring.size()];
            const Point2D& curr = ring[i];
            const Point2D& next = ring[(i + 1) % ring.size()];
            if (std::abs(cross(prev, curr, next)) < 1e-12) {
                changed = true;
                continue;
            }
            out.push_back(curr);
        }
        ring.swap(out);
    }
}

static Polygon2D normalizeOpenRing(const Polygon2D& ring) {
    Polygon2D out = ring;
    if (out.size() >= 2 && std::hypot(out.front().x - out.back().x, out.front().y - out.back().y) <= 1e-9) {
        out.pop_back();
    }
    return out;
}

static double pointSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq <= 1e-18) {
        return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    const double qx = a.x + t * dx;
    const double qy = a.y + t * dy;
    return std::hypot(p.x - qx, p.y - qy);
}

static double pointToRingDistance(const Point2D& p, const Polygon2D& ring) {
    const Polygon2D open = normalizeOpenRing(ring);
    if (open.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (open.size() == 1) {
        return std::hypot(p.x - open.front().x, p.y - open.front().y);
    }

    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < open.size(); ++i) {
        const Point2D& a = open[i];
        const Point2D& b = open[(i + 1) % open.size()];
        best = std::min(best, pointSegmentDistance(p, a, b));
    }
    return best;
}

static double maxRingToRingVertexDistance(const Polygon2D& src, const Polygon2D& dst) {
    const Polygon2D open = normalizeOpenRing(src);
    double worst = 0.0;
    for (const auto& p : open) {
        worst = std::max(worst, pointToRingDistance(p, dst));
    }
    return worst;
}

static double orient2D(const Point2D& a, const Point2D& b, const Point2D& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static bool pointOnSegment(const Point2D& p, const Point2D& a, const Point2D& b, double eps = 1e-9) {
    if (std::abs(orient2D(a, b, p)) > eps) {
        return false;
    }
    return p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
           p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
}

static bool segmentsIntersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D& b2) {
    const double o1 = orient2D(a1, a2, b1);
    const double o2 = orient2D(a1, a2, b2);
    const double o3 = orient2D(b1, b2, a1);
    const double o4 = orient2D(b1, b2, a2);
    const double eps = 1e-9;

    if (((o1 > eps && o2 < -eps) || (o1 < -eps && o2 > eps)) &&
        ((o3 > eps && o4 < -eps) || (o3 < -eps && o4 > eps))) {
        return true;
    }
    if (std::abs(o1) <= eps && pointOnSegment(b1, a1, a2, eps)) return true;
    if (std::abs(o2) <= eps && pointOnSegment(b2, a1, a2, eps)) return true;
    if (std::abs(o3) <= eps && pointOnSegment(a1, b1, b2, eps)) return true;
    if (std::abs(o4) <= eps && pointOnSegment(a2, b1, b2, eps)) return true;
    return false;
}

static bool ringHasSelfIntersection(const Polygon2D& ring) {
    const Polygon2D open = normalizeOpenRing(ring);
    if (open.size() < 4) {
        return false;
    }
    for (size_t i = 0; i < open.size(); ++i) {
        const Point2D& a1 = open[i];
        const Point2D& a2 = open[(i + 1) % open.size()];
        for (size_t j = i + 1; j < open.size(); ++j) {
            if (j == i) continue;
            if ((i + 1) % open.size() == j) continue;
            if (i == 0 && (j + 1) % open.size() == 0) continue;

            const Point2D& b1 = open[j];
            const Point2D& b2 = open[(j + 1) % open.size()];
            if (segmentsIntersect(a1, a2, b1, b2)) {
                return true;
            }
        }
    }
    return false;
}

static size_t strongestCornerIndex(const Polygon2D& ring) {
    const Polygon2D open = normalizeOpenRing(ring);
    if (open.size() < 3) {
        return 0;
    }
    size_t best_idx = 0;
    double best_turn = -1.0;
    for (size_t i = 0; i < open.size(); ++i) {
        const Point2D& prev = open[(i + open.size() - 1) % open.size()];
        const Point2D& curr = open[i];
        const Point2D& next = open[(i + 1) % open.size()];
        const double ax = curr.x - prev.x;
        const double ay = curr.y - prev.y;
        const double bx = next.x - curr.x;
        const double by = next.y - curr.y;
        const double an = std::hypot(ax, ay);
        const double bn = std::hypot(bx, by);
        if (an <= 1e-12 || bn <= 1e-12) {
            continue;
        }
        const double cross = (ax * by - ay * bx) / (an * bn);
        const double dot = (ax * bx + ay * by) / (an * bn);
        const double turn = std::abs(std::atan2(cross, dot));
        if (turn > best_turn) {
            best_turn = turn;
            best_idx = i;
        }
    }
    return best_idx;
}

static bool runIsStraightEnough(
    const Polygon2D& ring,
    size_t begin,
    size_t end,
    double max_residual_m,
    double monotonic_tol_m) {
    if (end <= begin + 1) {
        return false;
    }
    const Point2D& a = ring[begin];
    const Point2D& b = ring[end];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double chord = std::hypot(dx, dy);
    if (chord <= 1e-9) {
        return false;
    }
    const double ux = dx / chord;
    const double uy = dy / chord;
    double prev_t = -std::numeric_limits<double>::infinity();
    for (size_t i = begin; i <= end; ++i) {
        const double px = ring[i].x - a.x;
        const double py = ring[i].y - a.y;
        const double t = px * ux + py * uy;
        if (t + monotonic_tol_m < prev_t) {
            return false;
        }
        prev_t = t;
        if (i > begin && i < end && pointSegmentDistance(ring[i], a, b) > max_residual_m) {
            return false;
        }
    }
    return true;
}

static Polygon2D regularizeStraightSegments(const Polygon2D& ring, double cell_m) {
    Polygon2D open = normalizeOpenRing(ring);
    if (open.size() < 6 || cell_m <= 0.0) {
        return ring;
    }

    const size_t start_idx = strongestCornerIndex(open);
    Polygon2D ordered;
    ordered.reserve(open.size());
    for (size_t i = 0; i < open.size(); ++i) {
        ordered.push_back(open[(start_idx + i) % open.size()]);
    }

    const size_t min_run_points = 4;
    const double min_run_length_m = std::max(4.0 * cell_m, 0.35);
    const double max_residual_m = std::max(0.02, std::min(0.55 * cell_m, 0.05));
    const double monotonic_tol_m = std::max(1e-3, 0.10 * cell_m);

    Polygon2D simplified;
    simplified.reserve(ordered.size());
    size_t i = 0;
    while (i < ordered.size()) {
        if (simplified.empty() ||
            std::hypot(simplified.back().x - ordered[i].x, simplified.back().y - ordered[i].y) > 1e-9) {
            simplified.push_back(ordered[i]);
        }

        size_t best_end = i;
        double run_length_m = 0.0;
        for (size_t j = i + 1; j < ordered.size(); ++j) {
            run_length_m += std::hypot(ordered[j].x - ordered[j - 1].x, ordered[j].y - ordered[j - 1].y);
            if (j - i + 1 < min_run_points || run_length_m < min_run_length_m) {
                continue;
            }
            if (!runIsStraightEnough(ordered, i, j, max_residual_m, monotonic_tol_m)) {
                break;
            }
            best_end = j;
        }

        if (best_end > i + 1) {
            if (std::hypot(simplified.back().x - ordered[best_end].x,
                           simplified.back().y - ordered[best_end].y) > 1e-9) {
                simplified.push_back(ordered[best_end]);
            }
            i = best_end;
        } else {
            ++i;
        }
    }

    removeConsecutiveDuplicates(simplified);
    removeCollinear(simplified);
    if (simplified.size() < 3 || ringHasSelfIntersection(simplified)) {
        return ring;
    }

    const double base_area = polygonArea(open);
    const double new_area = polygonArea(simplified);
    const double abs_area_delta = std::abs(new_area - base_area);
    if (base_area > 1e-9) {
        const double rel_area_delta = abs_area_delta / base_area;
        if (rel_area_delta > 0.20 && abs_area_delta > std::max(2.0 * cell_m * cell_m, 0.05)) {
            return ring;
        }
    }

    const double max_dev_m = std::max(0.03, std::min(0.75 * cell_m, 0.08));
    const double deviation_m = std::max(
        maxRingToRingVertexDistance(open, simplified),
        maxRingToRingVertexDistance(simplified, open));
    if (deviation_m > max_dev_m) {
        return ring;
    }

    return simplified;
}

static bool ringInsideOuter(const Polygon2D& ring, const Polygon2D& outer) {
    const Polygon2D open = normalizeOpenRing(ring);
    if (open.empty()) {
        return false;
    }
    const size_t stride = std::max<size_t>(1, open.size() / 8);
    for (size_t i = 0; i < open.size(); i += stride) {
        if (!pointInPolyRayCast(open[i], outer)) {
            return false;
        }
    }
    return pointInPolyRayCast(open.back(), outer);
}

static ShapeContours regularizeShapeContours(ShapeContours shapes, double cell_m) {
    if (cell_m <= 0.0) {
        return shapes;
    }
    for (auto& shape : shapes) {
        const Polygon2D original_outer = shape.first;
        Polygon2D regularized_outer = regularizeStraightSegments(shape.first, cell_m);
        ensureCCW(regularized_outer);

        std::vector<Polygon2D> regularized_holes;
        regularized_holes.reserve(shape.second.size());
        bool keep_regularized_outer = true;
        for (const auto& original_hole : shape.second) {
            Polygon2D hole = regularizeStraightSegments(original_hole, cell_m);
            ensureCW(hole);
            if (!ringInsideOuter(hole, regularized_outer)) {
                hole = original_hole;
                ensureCW(hole);
                if (!ringInsideOuter(hole, regularized_outer)) {
                    keep_regularized_outer = false;
                }
            }
            regularized_holes.push_back(std::move(hole));
        }

        if (!keep_regularized_outer) {
            shape.first = original_outer;
            ensureCCW(shape.first);
            for (auto& hole : shape.second) {
                ensureCW(hole);
            }
            continue;
        }

        shape.first = std::move(regularized_outer);
        shape.second = std::move(regularized_holes);
    }
    return shapes;
}

static ShapeContours groupRingsIntoShapes(
    std::vector<Polygon2D> rings,
    double min_area_m2) {
    // Filter tiny/degenerate rings
    std::vector<Polygon2D> filtered;
    filtered.reserve(rings.size());
    for (auto& r : rings) {
        removeConsecutiveDuplicates(r);
        removeCollinear(r);
        double area = std::abs(signedArea2D(r));
        if (r.size() >= 3 && area >= min_area_m2) {
            filtered.push_back(std::move(r));
        }
    }
    rings = std::move(filtered);
    if (rings.empty()) return {};

    std::vector<double> areas(rings.size());
    for (size_t i = 0; i < rings.size(); ++i) {
        areas[i] = std::abs(signedArea2D(rings[i]));
    }
    std::vector<size_t> order(rings.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return areas[a] > areas[b];
    });

    std::vector<Polygon2D> rings_sorted;
    rings_sorted.reserve(rings.size());
    std::vector<double> areas_sorted;
    areas_sorted.reserve(rings.size());
    for (size_t idx : order) {
        rings_sorted.push_back(std::move(rings[idx]));
        areas_sorted.push_back(areas[idx]);
    }
    rings = std::move(rings_sorted);
    areas = std::move(areas_sorted);

    // Centroids = mean of vertices (matches python)
    std::vector<Point2D> centroids(rings.size());
    for (size_t i = 0; i < rings.size(); ++i) {
        double sx = 0.0, sy = 0.0;
        for (const auto& p : rings[i]) { sx += p.x; sy += p.y; }
        double inv = 1.0 / std::max<size_t>(1, rings[i].size());
        centroids[i] = Point2D(sx * inv, sy * inv);
    }

    std::vector<int> parent(rings.size(), -1);
    for (size_t i = 0; i < rings.size(); ++i) {
        int best = -1;
        double best_area = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < rings.size(); ++j) {
            if (areas[j] <= areas[i]) continue;
            if (!pointInPolyRayCast(centroids[i], rings[j])) continue;
            if (areas[j] < best_area) {
                best_area = areas[j];
                best = static_cast<int>(j);
            }
        }
        parent[i] = best;
    }

    std::vector<int> depth(rings.size(), 0);
    for (size_t i = 0; i < rings.size(); ++i) {
        int d = 0;
        int p = parent[i];
        while (p != -1) {
            d++;
            p = parent[static_cast<size_t>(p)];
        }
        depth[i] = d;
    }

    std::vector<std::pair<Polygon2D, std::vector<Polygon2D>>> shapes;
    for (size_t i = 0; i < rings.size(); ++i) {
        if (depth[i] % 2 != 0) continue;  // holes are odd depth
        Polygon2D outer = rings[i];
        ensureCCW(outer);
        std::vector<Polygon2D> holes;
        for (size_t j = 0; j < rings.size(); ++j) {
            if (parent[j] == static_cast<int>(i) && depth[j] == depth[i] + 1) {
                Polygon2D hole = rings[j];
                ensureCW(hole);
                holes.push_back(std::move(hole));
            }
        }
        shapes.emplace_back(std::move(outer), std::move(holes));
    }
    return shapes;
}

static int totalHoleCount(const ShapeContours& shapes) {
    int holes = 0;
    for (const auto& shape : shapes) {
        holes += static_cast<int>(shape.second.size());
    }
    return holes;
}

static bool sameShapeTopology(const ShapeContours& a, const ShapeContours& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].second.size() != b[i].second.size()) {
            return false;
        }
        const double area_a = polygonArea(a[i].first);
        const double area_b = polygonArea(b[i].first);
        const double denom = std::max({area_a, area_b, 1e-9});
        if (std::abs(area_a - area_b) / denom > 0.35) {
            return false;
        }
    }
    return totalHoleCount(a) == totalHoleCount(b);
}

static ShapeContours extractShapesFromOcc(const OccGrid& g, double min_area_m2) {
    if (g.w <= 0 || g.h <= 0 || g.occ.empty()) {
        return {};
    }

    ShapeContours baseline = groupRingsIntoShapes(extractContourRingsFromOcc(g), min_area_m2);
    ShapeContours chosen = baseline;

    // Only promote the signed-distance contour when it preserves the same topology
    // as the current binary pipeline; otherwise keep the existing behavior.
    if (!baseline.empty()) {
        ScalarGrid sdf = signedDistanceFieldFromOcc(g);
        if (!sdf.values.empty()) {
            ShapeContours refined = groupRingsIntoShapes(extractContourRingsFromSdf(sdf), min_area_m2);
            if (!refined.empty() && sameShapeTopology(refined, baseline)) {
                chosen = std::move(refined);
            }
        }
    }

    return regularizeShapeContours(std::move(chosen), g.cell);
}

static ShapeContours polygonizeClusterGridLikePython(
    const std::vector<Point2D>& pts2d,
    double grid_cell_m,
    double contour_cell_m,
    double inflate_radius_m,
    double smooth_radius_m,
    double min_contour_area_m2) {
    if (pts2d.empty() || grid_cell_m <= 0.0) return {};
    const double padding = std::max(0.20, 2.0 * inflate_radius_m);
    OccGrid occ = occupancyFromPoints(pts2d, grid_cell_m, padding);
    occ = inflateOccupancy(occ, inflate_radius_m);
    occ = closeOccupancyConservative(occ, smooth_radius_m);

    // Optional contour coarsening (conservative max-pooling), matching python's contour_cell.
    int factor = 1;
    if (contour_cell_m > grid_cell_m) {
        factor = std::max(1, static_cast<int>(std::lround(contour_cell_m / grid_cell_m)));
    }
    if (factor > 1) {
        occ = maxpoolOccupancy(occ, factor);
    }

    return extractShapesFromOcc(occ, min_contour_area_m2);
}

static std::vector<Obstacle2D> polygonizeBlockedCells(
    const PropagatedGrid& grid,
    double contour_cell_m,
    double inflate_radius_m,
    double smooth_radius_m,
    double geom_smooth_radius_m,
    double min_contour_area_m2,
    bool preserve_holes,
    double preserve_holes_min_area_m2,
    int* total_holes_out,
    double gradient_threshold = 0.0) {
    if (total_holes_out) {
        *total_holes_out = 0;
    }
    if (grid.w <= 0 || grid.h <= 0 || grid.cells.empty()) {
        return {};
    }

    std::vector<Obstacle2D> obstacles;
    obstacles.reserve(grid.cells.size());
    const double cell_min_area = std::max(0.0, min_contour_area_m2);
    for (int y = 0; y < grid.h; ++y) {
        for (int x = 0; x < grid.w; ++x) {
            const auto& cell = grid.cells[propagatedGridIndex(grid, x, y)];
            if (!cell.inside_scope) {
                continue;
            }
            if (cell.state != PropagatedCellState::MeasuredObstacle &&
                cell.state != PropagatedCellState::BlockedUnknown) {
                continue;
            }

            const ObstacleVisualType visual_type =
                (cell.state == PropagatedCellState::BlockedUnknown)
                    ? ObstacleVisualType::Unknown
                    : ObstacleVisualType::Known;
            Obstacle2D obstacle = gridCellRectObstacle(
                grid, x, y, visual_type, gradient_threshold, false);
            if (polygonArea(obstacle.outer) >= cell_min_area) {
                obstacles.push_back(std::move(obstacle));
            }
        }
    }

    (void)contour_cell_m;
    (void)inflate_radius_m;
    (void)smooth_radius_m;
    (void)geom_smooth_radius_m;
    (void)preserve_holes;
    (void)preserve_holes_min_area_m2;
    return obstacles;
}

static ObstacleDetectionResult detectObstaclesPatchworkBundleImpl(
    const PointCloudPtr& loaded_cloud,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params) {
    ObstacleDetectionResult res;
    auto abortIfCancelled = [&res]() -> bool {
        if (!g_obstacleCancelCallback || !g_obstacleCancelCallback()) {
            return false;
        }
        res.error_message = "Operation cancelled";
        return true;
    };

    if (!loaded_cloud || loaded_cloud->empty()) {
        res.error_message = "No point cloud loaded.";
        return res;
    }
    if (params.source_path.empty()) {
        res.error_message = "Patchwork method requires params.source_path to be set.";
        return res;
    }

    const PatchworkBundlePaths bundle = derivePatchworkBundlePaths(params.source_path);
    PointCloudPtr nonground_cloud;
    try {
        if (fileExists(bundle.nonground_path) && bundle.nonground_path != params.source_path) {
            nonground_cloud = loadPointCloudFile(bundle.nonground_path);
        } else if (params.source_path.find("nonground") != std::string::npos) {
            nonground_cloud = loaded_cloud;
        } else if (fileExists(bundle.nonground_path)) {
            nonground_cloud = loadPointCloudFile(bundle.nonground_path);
        }
    } catch (const std::exception& ex) {
        res.error_message = ex.what();
        return res;
    }

    if (!nonground_cloud || nonground_cloud->empty()) {
        res.error_message =
            "Patchwork detector could not find a non-ground companion cloud. "
            "Load a corrected_patchwork_scores_* or corrected_patchwork_nonground_* file.";
        return res;
    }

    size_t score_point_count = 0;
    double mean_ground_probability = 0.0;
    try {
        if (fileExists(bundle.scores_path)) {
            auto score_cloud = loadPointCloudFileXYZI(bundle.scores_path);
            score_point_count = score_cloud->size();
            double intensity_sum = 0.0;
            size_t valid_scores = 0;
            for (const auto& pt : score_cloud->points) {
                if (!std::isfinite(pt.intensity)) {
                    continue;
                }
                intensity_sum += static_cast<double>(pt.intensity);
                ++valid_scores;
            }
            if (valid_scores > 0) {
                mean_ground_probability = intensity_sum / static_cast<double>(valid_scores);
            }
        }
    } catch (const std::exception& ex) {
        std::cout << "[PatchworkDetect] Score-cloud load skipped: " << ex.what() << std::endl;
    }

    const Polygon2D scope = effectiveScopePolygon(roi_or_boundary);
    const double scope_margin_m =
        (scope.size() >= 3) ? std::max(0.0, params.scope_margin_m) : 0.0;
    PointCloudPtr scoped_display_cloud = filterCloudToPolygon(loaded_cloud, scope, scope_margin_m);
    PointCloudPtr scoped_nonground_cloud = filterCloudToPolygon(nonground_cloud, scope, scope_margin_m);
    res.stats.input_points = loaded_cloud->size();
    res.stats.roi_points = scoped_display_cloud ? scoped_display_cloud->size() : 0;
    res.stats.path_poses = 0;
    res.stats.raw_obstacle_candidates = scoped_nonground_cloud ? scoped_nonground_cloud->size() : 0;
    res.stats.obstacle_points_after_outlier = res.stats.raw_obstacle_candidates;
    if (abortIfCancelled()) {
        return res;
    }

    std::cout << "[PatchworkDetect] source=" << params.source_path
              << " score_points=" << score_point_count
              << " display_points=" << res.stats.roi_points
              << " nonground_points=" << res.stats.raw_obstacle_candidates
              << " mean_ground_probability=" << mean_ground_probability
              << std::endl;

    if (!scoped_nonground_cloud || scoped_nonground_cloud->empty()) {
        res.success = true;
        return res;
    }

    double grid_cell_m = params.grid_cell_m;
    if (grid_cell_m <= 0.0) {
        grid_cell_m = 0.09;
    }
    double contour_cell_m = params.contour_cell_m;
    if (contour_cell_m < 0.0) {
        contour_cell_m = 2.0 * grid_cell_m;
    }
    if (contour_cell_m < grid_cell_m) {
        contour_cell_m = grid_cell_m;
    }
    double smooth_radius_m = params.smooth_radius_m;
    if (smooth_radius_m < 0.0) {
        smooth_radius_m = 2.0 * grid_cell_m;
    }
    if (smooth_radius_m < 0.0) {
        smooth_radius_m = 0.0;
    }
    const double inflate_radius_m = std::max(0.0, params.inflate_radius_m);
    const double geom_smooth_radius_m = std::max(0.0, params.geom_smooth_radius_m);
    const bool preserve_holes = params.preserve_holes;
    const double preserve_holes_min_area_m2 = std::max(0.0, params.preserve_holes_min_area_m2);

    // Keep the Patchwork path raw for inspection: no extra denoising before clustering.
    PointCloudPtr obstacle_clean = scoped_nonground_cloud;

    std::vector<Point2D> obs_xy;
    obs_xy.reserve(obstacle_clean->size());
    size_t obs_xy_idx = 0;
    for (const auto& pt : obstacle_clean->points) {
        if ((obs_xy_idx++ & 0x1FFFu) == 0u && abortIfCancelled()) {
            return res;
        }
        obs_xy.emplace_back(pt.x, pt.y);
    }
    std::vector<int> labels = dbscan2D(obs_xy, params.cluster_eps_m, params.cluster_min_pts);
    if (abortIfCancelled()) {
        return res;
    }
    int max_label = -1;
    for (int l : labels) max_label = std::max(max_label, l);
    const int n_clusters = max_label + 1;
    res.stats.clusters_found = n_clusters;

    std::vector<std::vector<Point2D>> clusters;
    clusters.resize(static_cast<size_t>(n_clusters));
    std::vector<Point2D> noise_pts;
    noise_pts.reserve(obs_xy.size());
    for (size_t i = 0; i < obs_xy.size(); ++i) {
        if ((i & 0x1FFFu) == 0u && abortIfCancelled()) {
            return res;
        }
        int l = labels[i];
        if (l < 0) {
            noise_pts.push_back(obs_xy[i]);
            continue;
        }
        clusters[static_cast<size_t>(l)].push_back(obs_xy[i]);
    }

    std::vector<std::vector<Point2D>> cluster_list;
    cluster_list.reserve(clusters.size());
    for (auto& c : clusters) {
        if (!c.empty()) cluster_list.push_back(std::move(c));
    }
    clusters.clear();

    std::vector<Obstacle2D> micro_obstacles;
    if (params.micro_enable && !obs_xy.empty()) {
        std::vector<std::vector<Point2D>> normal_clusters;
        normal_clusters.reserve(cluster_list.size());
        size_t cluster_idx = 0;
        for (auto& cl : cluster_list) {
            if ((cluster_idx++ & 0x3Fu) == 0u && abortIfCancelled()) {
                return res;
            }
            if (isMicroCluster(cl, params)) {
                Obstacle2D obs;
                obs.outer = rectFromBbox(cl, params.micro_min_size_m, params.micro_margin_m);
                micro_obstacles.push_back(std::move(obs));
            } else {
                normal_clusters.push_back(std::move(cl));
            }
        }
        cluster_list = std::move(normal_clusters);

        std::vector<Point2D> micro_noise_pts;
        micro_noise_pts.reserve(obs_xy.size());
        for (size_t i = 0; i < obs_xy.size(); ++i) {
            if (labels[i] == -1) {
                micro_noise_pts.push_back(obs_xy[i]);
            }
        }
        if (static_cast<int>(micro_noise_pts.size()) >= params.micro_min_pts &&
            params.micro_noise_eps_m > 0.0) {
            std::vector<int> micro_labels = dbscan2D(
                micro_noise_pts, params.micro_noise_eps_m, params.micro_min_pts);
            if (abortIfCancelled()) {
                return res;
            }
            int micro_max_label = -1;
            for (int l : micro_labels) micro_max_label = std::max(micro_max_label, l);
            const int micro_n_clusters = micro_max_label + 1;
            if (micro_n_clusters > 0) {
                std::vector<std::vector<Point2D>> micro_clusters(static_cast<size_t>(micro_n_clusters));
                for (size_t i = 0; i < micro_noise_pts.size(); ++i) {
                    int l = micro_labels[i];
                    if (l < 0) continue;
                    micro_clusters[static_cast<size_t>(l)].push_back(micro_noise_pts[i]);
                }
                for (auto& mc : micro_clusters) {
                    if (mc.empty()) continue;
                    if (!isMicroCluster(mc, params)) continue;
                    Obstacle2D obs;
                    obs.outer = rectFromBbox(mc, params.micro_min_size_m, params.micro_margin_m);
                    micro_obstacles.push_back(std::move(obs));
                }
            }
        }
    }

    if (cluster_list.empty()) {
        res.stats.total_holes = 0;
        res.stats.obstacle_shapes = static_cast<int>(micro_obstacles.size());
        res.obstacles = std::move(micro_obstacles);
        res.success = true;
        return res;
    }

    const double merge_d2 = params.merge_distance_m * params.merge_distance_m;

    struct ClusterKD {
        std::vector<Point2D> pts;
        Point2D minp{0, 0};
        Point2D maxp{0, 0};
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
        std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> tree;
    };

    std::vector<ClusterKD> ck;
    ck.reserve(cluster_list.size());
    for (auto& c : cluster_list) {
        ClusterKD entry;
        entry.pts = std::move(c);
        entry.minp = entry.maxp = entry.pts.front();
        for (const auto& p : entry.pts) {
            entry.minp.x = std::min(entry.minp.x, p.x);
            entry.minp.y = std::min(entry.minp.y, p.y);
            entry.maxp.x = std::max(entry.maxp.x, p.x);
            entry.maxp.y = std::max(entry.maxp.y, p.y);
        }
        entry.pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        entry.pcl_cloud->reserve(entry.pts.size());
        for (const auto& p : entry.pts) {
            entry.pcl_cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x), static_cast<float>(p.y), 0.0f));
        }
        entry.tree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        entry.tree->setInputCloud(entry.pcl_cloud);
        ck.push_back(std::move(entry));
    }

    struct UnionFind {
        std::vector<int> parent;
        explicit UnionFind(int n) : parent(static_cast<size_t>(n)) {
            for (int i = 0; i < n; ++i) parent[static_cast<size_t>(i)] = i;
        }
        int find(int x) {
            int r = x;
            while (parent[static_cast<size_t>(r)] != r) {
                r = parent[static_cast<size_t>(r)];
            }
            while (parent[static_cast<size_t>(x)] != x) {
                int px = parent[static_cast<size_t>(x)];
                parent[static_cast<size_t>(x)] = r;
                x = px;
            }
            return r;
        }
        void unite(int a, int b) {
            int ra = find(a);
            int rb = find(b);
            if (ra != rb) parent[static_cast<size_t>(ra)] = rb;
        }
    };

    UnionFind uf(static_cast<int>(ck.size()));

    auto aabbMinDist2 = [](const ClusterKD& a, const ClusterKD& b) -> double {
        double dx = 0.0;
        if (a.maxp.x < b.minp.x) dx = b.minp.x - a.maxp.x;
        else if (b.maxp.x < a.minp.x) dx = a.minp.x - b.maxp.x;
        double dy = 0.0;
        if (a.maxp.y < b.minp.y) dy = b.minp.y - a.maxp.y;
        else if (b.maxp.y < a.minp.y) dy = a.minp.y - b.maxp.y;
        return dx * dx + dy * dy;
    };

    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist2(1);

    for (int i = 0; i < static_cast<int>(ck.size()); ++i) {
        if ((i & 0x0Fu) == 0 && abortIfCancelled()) {
            return res;
        }
        for (int j = i + 1; j < static_cast<int>(ck.size()); ++j) {
            if (aabbMinDist2(ck[static_cast<size_t>(i)], ck[static_cast<size_t>(j)]) > merge_d2) {
                continue;
            }
            double best_d2 = std::numeric_limits<double>::infinity();
            for (const auto& p : ck[static_cast<size_t>(i)].pcl_cloud->points) {
                int found = ck[static_cast<size_t>(j)].tree->nearestKSearch(p, 1, nn_idx, nn_dist2);
                if (found > 0) {
                    best_d2 = std::min(best_d2, static_cast<double>(nn_dist2[0]));
                    if (best_d2 <= merge_d2) break;
                }
            }
            if (best_d2 <= merge_d2) {
                uf.unite(i, j);
            }
        }
    }

    std::unordered_map<int, std::vector<int>> groups;
    groups.reserve(ck.size());
    for (int i = 0; i < static_cast<int>(ck.size()); ++i) {
        groups[uf.find(i)].push_back(i);
    }
    res.stats.groups_merged = static_cast<int>(groups.size());

    std::vector<Obstacle2D> obstacles_out;
    for (const auto& kv : groups) {
        if (abortIfCancelled()) {
            return res;
        }
        const auto& idxs = kv.second;
        std::vector<Point2D> merged_pts;
        size_t total_pts = 0;
        for (int ci : idxs) total_pts += ck[static_cast<size_t>(ci)].pts.size();
        merged_pts.reserve(total_pts);
        for (int ci : idxs) {
            const auto& pts = ck[static_cast<size_t>(ci)].pts;
            merged_pts.insert(merged_pts.end(), pts.begin(), pts.end());
        }
        if (merged_pts.empty()) continue;

        Polygon2D merged_hull;
        try {
            if (merged_pts.size() >= 3) {
                merged_hull = computeConvexHull(merged_pts);
            } else {
                merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
            }
        } catch (...) {
            merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
        }
        double hull_area = polygonArea(merged_hull);
        if (hull_area <= 1e-12) {
            merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
            hull_area = polygonArea(merged_hull);
        }

        bool use_grid = false;
        bool allow_grid = (params.polygon_mode == ObstaclePolygonMode::Auto ||
                           params.polygon_mode == ObstaclePolygonMode::Grid);

        if (allow_grid) {
            const bool prefer_grid = (params.polygon_mode == ObstaclePolygonMode::Auto) &&
                (geom_smooth_radius_m > 0.0 || contour_cell_m > grid_cell_m + 1e-12);

            bool hollow_trigger = false;
            if (grid_cell_m > 0.0 && hull_area > params.min_contour_area_m2) {
                OccGrid occ0 = occupancyFromPoints(merged_pts, grid_cell_m, 0.20);
                size_t occ_sum = 0;
                for (uint8_t v : occ0.occ) occ_sum += (v != 0);
                const double occ_area = static_cast<double>(occ_sum) * (grid_cell_m * grid_cell_m);
                const double hollow_ratio = occ_area / std::max(hull_area, 1e-9);
                hollow_trigger = (hollow_ratio < params.hollow_ratio_thresh);
            }

            use_grid = (params.polygon_mode == ObstaclePolygonMode::Grid) || prefer_grid || hollow_trigger;
        }

        if (use_grid) {
            auto shapes = polygonizeClusterGrid(
                merged_pts,
                grid_cell_m,
                contour_cell_m,
                inflate_radius_m,
                /*smooth_radius_m=*/0.0,
                params.min_contour_area_m2);

            if (!shapes.empty()) {
                auto pointInAnyShape = [&](const Point2D& p) -> bool {
                    for (const auto& sh : shapes) {
                        if (!pointInPolyRayCast(p, sh.first)) continue;
                        bool in_hole = false;
                        for (const auto& h : sh.second) {
                            if (pointInPolyRayCast(p, h)) {
                                in_hole = true;
                                break;
                            }
                        }
                        if (!in_hole) return true;
                    }
                    return false;
                };

                for (int ci : idxs) {
                    const auto& cpts = ck[static_cast<size_t>(ci)].pts;
                    if (cpts.empty()) continue;
                    bool covered = false;
                    const size_t stride = std::max<size_t>(1, cpts.size() / 12);
                    for (size_t k = 0; k < cpts.size(); k += stride) {
                        if (pointInAnyShape(cpts[k])) {
                            covered = true;
                            break;
                        }
                    }
                    if (!covered && pointInAnyShape(cpts.back())) {
                        covered = true;
                    }
                    if (covered) continue;

                    Polygon2D tiny = rectFromBbox(cpts, std::max(0.02, 0.75 * grid_cell_m), 0.0);
                    shapes.emplace_back(std::move(tiny), std::vector<Polygon2D>{});
                }

                for (auto& sh : shapes) {
                    Obstacle2D obs;
                    obs.outer = std::move(sh.first);
                    obs.holes = std::move(sh.second);
                    obstacles_out.push_back(std::move(obs));
                }
                continue;
            }
        }

        Obstacle2D obs;
        obs.outer = std::move(merged_hull);
        obstacles_out.push_back(std::move(obs));
    }

    if (geom_smooth_radius_m > 0.0 && !obstacles_out.empty()) {
        obstacles_out = smoothShapesRollingDiskGrid(
            obstacles_out,
            geom_smooth_radius_m,
            grid_cell_m,
            contour_cell_m,
            params.min_contour_area_m2,
            preserve_holes,
            preserve_holes_min_area_m2);
    }
    if (abortIfCancelled()) {
        return res;
    }

    if (!micro_obstacles.empty()) {
        obstacles_out.reserve(obstacles_out.size() + micro_obstacles.size());
        for (auto& m : micro_obstacles) {
            obstacles_out.push_back(std::move(m));
        }
    }

    int final_holes = 0;
    for (const auto& o : obstacles_out) {
        final_holes += static_cast<int>(o.holes.size());
    }
    res.stats.total_holes = final_holes;
    res.stats.obstacle_shapes = static_cast<int>(obstacles_out.size());
    res.obstacles = std::move(obstacles_out);
    res.success = true;
    return res;
}

}  // namespace

ObstacleDetectionResult detectObstaclesAuto(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& driven_path,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params) {
    ObstacleDetectionResult res;
    auto abortIfCancelled = [&res]() -> bool {
        if (!g_obstacleCancelCallback || !g_obstacleCancelCallback()) {
            return false;
        }
        res.error_message = "Operation cancelled";
        return true;
    };
    if (!cloud || cloud->empty()) {
        res.error_message = "No point cloud loaded.";
        return res;
    }

    const Polygon2D scope = effectiveScopePolygon(roi_or_boundary);
    const double scope_margin_m =
        (scope.size() >= 3) ? std::max(0.0, params.scope_margin_m) : 0.0;
    PointCloudPtr scoped_cloud = filterCloudToPolygon(cloud, scope, scope_margin_m);
    res.stats.input_points = cloud->size();
    res.stats.roi_points = scoped_cloud ? scoped_cloud->size() : 0;

    // Filter path to ROI (if provided)
    std::vector<PathState> path = driven_path;
    if (!scope.empty()) {
        path = filterPathToPolygon(driven_path, scope, scope_margin_m);
    }
    res.stats.path_poses = path.size();
    if (abortIfCancelled()) {
        return res;
    }

    // Derived defaults (match Python main()).
    double grid_cell_m = params.grid_cell_m;
    if (grid_cell_m <= 0.0) {
        grid_cell_m = 0.09;
    }
    double contour_cell_m = params.contour_cell_m;
    if (contour_cell_m < 0.0) {
        contour_cell_m = 2.0 * grid_cell_m;
    }
    if (contour_cell_m < grid_cell_m) {
        contour_cell_m = grid_cell_m;
    }
    double smooth_radius_m = params.smooth_radius_m;
    if (smooth_radius_m < 0.0) {
        smooth_radius_m = 2.0 * grid_cell_m;
    }
    if (smooth_radius_m < 0.0) {
        smooth_radius_m = 0.0;
    }
    const double inflate_radius_m = std::max(0.0, params.inflate_radius_m);
    const double geom_smooth_radius_m = std::max(0.0, params.geom_smooth_radius_m);
    const bool preserve_holes = params.preserve_holes;
    const double preserve_holes_min_area_m2 = std::max(0.0, params.preserve_holes_min_area_m2);
    PointCloudPtr obstacle_raw(new PointCloud);
    bool have_preclassified_obstacles = false;

    if (params.detection_method == ObstacleDetectionMethod::ClothSimulationFilter) {
        PointCloudPtr csf_input_cloud = scoped_cloud;
        if (params.csf_pre_sor_enabled) {
            const size_t before_pre_sor = csf_input_cloud ? csf_input_cloud->size() : 0;
            csf_input_cloud = removeStatisticalOutliersMeanDist(
                csf_input_cloud,
                params.csf_pre_sor_k,
                params.csf_pre_sor_std);
            if (abortIfCancelled()) {
                return res;
            }
            std::cout << "[ObstacleDetect] CSF: pre-SOR input_points=" << before_pre_sor
                      << ", output_points=" << (csf_input_cloud ? csf_input_cloud->size() : 0)
                      << ", k=" << params.csf_pre_sor_k
                      << ", std=" << params.csf_pre_sor_std
                      << "\n";
        }
        std::cout << "[ObstacleDetect] CSF: segmenting ground/non-ground"
                  << " (cloth_resolution=" << params.csf_cloth_resolution_m
                  << ", max_iterations=" << params.csf_max_iterations
                  << ", classification_threshold=" << params.csf_classification_threshold_m
                  << ", rigidness=" << params.csf_rigidness
                  << ", slope_processing=" << (params.csf_slope_processing ? "on" : "off")
                  << ", pre_sor=" << (params.csf_pre_sor_enabled ? "on" : "off")
                  << ", trail_cleanup=" << (params.csf_trail_footprint_cleanup ? "on" : "off")
                  << ", trail_margin=" << params.csf_trail_cleanup_margin_m
                  << ")\n";
        const CsfSegmentationResult csf = segmentGroundClothSimulation(
            csf_input_cloud,
            params,
            abortIfCancelled);
        if (abortIfCancelled()) {
            return res;
        }
        if (!csf.ground || csf.ground->empty()) {
            res.error_message = "CSF could not classify any ground points.";
            return res;
        }
        obstacle_raw = csf.nonground ? csf.nonground : PointCloudPtr(new PointCloud);
        res.csf_ground_cloud = csf.ground;
        res.csf_nonground_cloud = csf.nonground;
        res.stats.footprint_ground_points = csf.ground->size();
        res.stats.ground_points_band = csf.ground->size();
        res.stats.raw_obstacle_candidates = obstacle_raw->size();
        res.stats.anchor_cells = csf.grid_cells;
        have_preclassified_obstacles = true;
        std::cout << "[ObstacleDetect] CSF: ground=" << csf.ground->size()
                  << ", nonground=" << (csf.nonground ? csf.nonground->size() : 0)
                  << ", cloth_cells=" << csf.grid_cells << "\n";
    }

    // ------------------------------------------------------------------
    // 3. Ground detection
    // ------------------------------------------------------------------
    if (!have_preclassified_obstacles) {
    PointCloudPtr fp_ground(new PointCloud);
    const bool use_relaxed_ground_support =
        params.ground_model_mode == GroundModelMode::LocalHeightField ||
        params.ground_model_mode == GroundModelMode::PropagatedGrid ||
        params.ground_model_mode == GroundModelMode::GroundZGradientGrid;
    if (!path.empty()) {
        if (use_relaxed_ground_support) {
            PointCloudPtr fp_ground_strict = extractFootprintGround(
                scoped_cloud, path,
                params.robot_length_m, params.robot_width_m, params.footprint_margin_m,
                params.ground_z_max, true);
            PointCloudPtr fp_ground_relaxed = extractFootprintGround(
                scoped_cloud, path,
                params.robot_length_m, params.robot_width_m, params.footprint_margin_m,
                params.ground_z_max, false);

            const bool use_grid_cell =
                params.ground_model_mode == GroundModelMode::PropagatedGrid ||
                params.ground_model_mode == GroundModelMode::GroundZGradientGrid;
            const double grid_seed_quantile =
                params.ground_model_mode == GroundModelMode::GroundZGradientGrid
                    ? groundZLowQuantile(params)
                    : std::clamp(params.raw_cell_low_quantile, 0.0, 0.50);
            PointCloudPtr strict_seed_preview = aggregateGroundSeedsQuantileXY(
                fp_ground_strict,
                use_grid_cell ? grid_cell_m : params.local_ground_cell_m,
                use_grid_cell ? grid_seed_quantile : kLocalGroundSeedQuantile);
            PointCloudPtr relaxed_seed_preview = aggregateGroundSeedsQuantileXY(
                fp_ground_relaxed,
                use_grid_cell ? grid_cell_m : params.local_ground_cell_m,
                use_grid_cell ? grid_seed_quantile : kLocalGroundSeedQuantile);
            const size_t strict_cells = strict_seed_preview ? strict_seed_preview->size() : 0;
            const size_t relaxed_cells = relaxed_seed_preview ? relaxed_seed_preview->size() : 0;
            const size_t min_seed_cells = static_cast<size_t>(std::max(3, params.local_ground_min_pts));
            const bool use_strict_support =
                strict_cells >= min_seed_cells &&
                (relaxed_cells == 0 || (10 * strict_cells) >= (6 * relaxed_cells));
            fp_ground = use_strict_support ? fp_ground_strict : fp_ground_relaxed;

            std::cout << "[ObstacleDetect] Ground: using footprint path ("
                      << (use_strict_support ? "z-capped" : "relaxed")
                      << ", poses=" << path.size()
                      << ", points=" << fp_ground->size()
                      << ", support_cells="
                      << (use_strict_support ? strict_cells : relaxed_cells)
                      << ", z_max=" << params.ground_z_max << ")\n";
        } else {
            fp_ground = extractFootprintGround(
                scoped_cloud, path,
                params.robot_length_m, params.robot_width_m, params.footprint_margin_m,
                params.ground_z_max, true);
        }
        if (fp_ground->size() < 10) {
            // Fallback: z-threshold
            std::cout << "[ObstacleDetect] Ground: footprint sample too small ("
                      << fp_ground->size() << "), falling back to z-threshold (z <= "
                      << params.ground_z_max << ")\n";
            fp_ground->clear();
            size_t ground_idx = 0;
            for (const auto& pt : scoped_cloud->points) {
                if ((ground_idx++ & 0x1FFFu) == 0u && abortIfCancelled()) {
                    return res;
                }
                if (pt.z <= params.ground_z_max) fp_ground->push_back(pt);
            }
        } else if (!use_relaxed_ground_support) {
            std::cout << "[ObstacleDetect] Ground: using footprint path (poses="
                      << path.size() << ", points=" << fp_ground->size()
                      << ", z_max=" << params.ground_z_max << ")\n";
        }
    } else {
        std::cout << "[ObstacleDetect] Ground: no path provided, using z-threshold (z <= "
                  << params.ground_z_max << ")\n";
        size_t ground_idx = 0;
        for (const auto& pt : scoped_cloud->points) {
            if ((ground_idx++ & 0x1FFFu) == 0u && abortIfCancelled()) {
                return res;
            }
            if (pt.z <= params.ground_z_max) fp_ground->push_back(pt);
        }
    }
    res.stats.footprint_ground_points = fp_ground->size();
    if (abortIfCancelled()) {
        return res;
    }

    PlaneModel plane;
    LocalHeightFieldModel local_ground;
    if (params.ground_model_mode == GroundModelMode::LocalHeightField) {
        std::cout << "[ObstacleDetect] Ground: building local height field (n="
                  << fp_ground->size() << ", cell=" << params.local_ground_cell_m
                  << ", knn=" << params.local_ground_knn
                  << ", radius=" << params.local_ground_radius_m << ")\n";
        local_ground = buildLocalHeightFieldModel(fp_ground, params);
        plane = local_ground.fallback_plane;
        std::cout << "[ObstacleDetect] Ground: local height field "
                  << (local_ground.valid ? "enabled" : "falling back to plane")
                  << " (support="
                  << (local_ground.xy_support ? local_ground.xy_support->size() : 0)
                  << ", mode=" << groundModelName(params.ground_model_mode) << ")\n";
    } else if (params.ground_model_mode == GroundModelMode::PropagatedGrid ||
               params.ground_model_mode == GroundModelMode::GroundZGradientGrid) {
        const double plane_seed_quantile =
            params.ground_model_mode == GroundModelMode::GroundZGradientGrid
                ? groundZLowQuantile(params)
                : kLocalGroundSeedQuantile;
        PointCloudPtr plane_support = aggregateGroundSeedsQuantileXY(
            fp_ground, grid_cell_m, plane_seed_quantile);
        plane = fallbackPlaneFromGround(
            (plane_support && !plane_support->empty()) ? plane_support : fp_ground,
            params.ransac_iters,
            params.ransac_thresh_m);
        std::cout << "[ObstacleDetect] Ground: building " << groundModelName(params.ground_model_mode)
                  << " (n="
                  << scoped_cloud->size() << ", cell=" << grid_cell_m
                  << ", scope_margin=" << scope_margin_m
                  << ", mode=" << groundModelName(params.ground_model_mode) << ")\n";
    } else if (fp_ground->size() < 20) {
        std::cout << "[ObstacleDetect] Ground: using median-Z flat plane (n="
                  << fp_ground->size() << ")\n";
        plane = flatPlaneAtZ(medianZ(fp_ground));
    } else {
        std::cout << "[ObstacleDetect] Ground: fitting RANSAC plane (n="
                  << fp_ground->size() << ", iters=" << params.ransac_iters
                  << ", thresh=" << params.ransac_thresh_m << ")\n";
        plane = fitPlaneRansac(fp_ground, params.ransac_iters, params.ransac_thresh_m);
    }
    res.stats.plane_nx = plane.nx;
    res.stats.plane_ny = plane.ny;
    res.stats.plane_nz = plane.nz;
    res.stats.plane_d = plane.d;

    if (params.ground_model_mode == GroundModelMode::GroundZGradientGrid) {
        PropagatedGrid grid =
            buildPropagatedGrid(scoped_cloud, scope, grid_cell_m, scope_margin_m, params);
        if (grid.cells.empty()) {
            res.error_message = "Unable to build ground-Z gradient detector grid.";
            return res;
        }

        initializeGroundZMap(&grid, params);
        const size_t filled_empty_cells = fillEmptyGroundZCellsFromNeighbours(&grid, params);
        res.stats.filled_empty_cells = filled_empty_cells;

        markTrailCoveredCells(
            &grid,
            path,
            params.robot_length_m,
            params.robot_width_m,
            params.footprint_margin_m);
        seedGroundZGradientReachability(&grid);

        const GroundZGradientStats gradient_stats =
            computeGroundZGradientStats(grid, params);
        classifyGroundZGradientGrid(&grid, gradient_stats, params, &res.stats);
        res.stats.filled_empty_cells = filled_empty_cells;
        res.debug_grid_cells = buildGroundDebugCells(
            grid,
            gradient_stats.threshold,
            params.min_contour_area_m2);

        int total_holes = 0;
        res.obstacles = polygonizeBlockedCells(
            grid,
            contour_cell_m,
            inflate_radius_m,
            smooth_radius_m,
            geom_smooth_radius_m,
            params.min_contour_area_m2,
            preserve_holes,
            preserve_holes_min_area_m2,
            &total_holes,
            gradient_stats.threshold);
        res.stats.total_holes = total_holes;
        res.stats.obstacle_shapes = static_cast<int>(res.obstacles.size());
        std::cout << "[ObstacleDetect] Ground-Z gradient grid: anchors="
                  << res.stats.anchor_cells
                  << ", reachable_ground=" << res.stats.propagated_ground_cells
                  << ", measured_obstacle=" << res.stats.measured_obstacle_cells
                  << ", blocked_unknown=" << res.stats.blocked_unknown_cells
                  << ", filled_empty=" << res.stats.filled_empty_cells
                  << ", gradient_threshold=" << gradient_stats.threshold
                  << ", high_gradient_edges=" << gradient_stats.high_gradient_edges
                  << "/" << gradient_stats.edge_count
                  << ", gradient_median=" << gradient_stats.median_gradient
                  << ", gradient_mad=" << gradient_stats.mad_gradient
                  << ", shapes=" << res.obstacles.size()
                  << "\n";
        res.success = true;
        return res;
    }

    if (params.ground_model_mode == GroundModelMode::PropagatedGrid) {
        PropagatedGrid grid =
            buildPropagatedGrid(scoped_cloud, scope, grid_cell_m, scope_margin_m, params);
        if (grid.cells.empty()) {
            res.error_message = "Unable to build propagated detector grid.";
            return res;
        }

        PointCloudPtr anchor_seeds = aggregateGroundSeedsQuantileXY(
            fp_ground,
            grid_cell_m,
            std::clamp(params.raw_cell_low_quantile, 0.0, 0.50));
        if (anchor_seeds) {
            for (const auto& pt : anchor_seeds->points) {
                const int gx = static_cast<int>(std::floor((pt.x - grid.xmin) / grid.cell));
                const int gy = static_cast<int>(std::floor((pt.y - grid.ymin) / grid.cell));
                if (!propagatedGridInBounds(grid, gx, gy)) {
                    continue;
                }
                auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
                if (!cell.inside_scope) {
                    continue;
                }
                cell.anchor_z_values.push_back(pt.z);
            }
        }

        markTrailCoveredCells(
            &grid,
            path,
            params.robot_length_m,
            params.robot_width_m,
            params.footprint_margin_m);

        size_t anchor_cells = 0;
        for (auto& cell : grid.cells) {
            if (!cell.anchor_z_values.empty()) {
                cell.trail_covered = true;
            }
            if (!cell.inside_scope || !cell.trail_covered) {
                continue;
            }
            double anchor_z = 0.0;
            double anchor_confidence = 0.0;
            if (!estimateObservedAnchorGroundZ(cell, &anchor_z, &anchor_confidence)) {
                continue;
            }
            const bool had_anchor_seed = !cell.anchor_z_values.empty();
            cell.state = PropagatedCellState::AnchorGround;
            cell.ground_z = anchor_z;
            cell.confidence = anchor_confidence;
            if (cell.anchor_z_values.empty()) {
                cell.anchor_z_values.push_back(static_cast<float>(anchor_z));
            }
            cell.reason = had_anchor_seed ? "trail_anchor_seed" : "trail_anchor_observed";
            anchor_cells++;
        }

        bool trail_changed = true;
        for (int pass = 0; pass < 4 && trail_changed; ++pass) {
            trail_changed = false;
            for (int gy = 0; gy < grid.h; ++gy) {
                for (int gx = 0; gx < grid.w; ++gx) {
                    auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
                    if (!cell.inside_scope || !cell.trail_covered ||
                        cell.state != PropagatedCellState::Unknown) {
                        continue;
                    }
                    const PropagationEstimate estimate =
                        estimateGroundFromNeighbours(grid, gx, gy, params);
                    if (!estimate.valid) {
                        continue;
                    }
                    cell.state = PropagatedCellState::AnchorGround;
                    cell.ground_z = estimate.predicted_z;
                    cell.confidence = std::max(2.5, estimate.confidence);
                    cell.anchor_z_values.push_back(static_cast<float>(cell.ground_z));
                    cell.reason = "trail_anchor_interpolated";
                    anchor_cells++;
                    trail_changed = true;
                }
            }
        }

        for (int gy = 0; gy < grid.h; ++gy) {
            for (int gx = 0; gx < grid.w; ++gx) {
                auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
                if (!cell.inside_scope || !cell.trail_covered ||
                    cell.state != PropagatedCellState::Unknown) {
                    continue;
                }
                const Point2D center = propagatedGridCellCenter(grid, gx, gy);
                cell.state = PropagatedCellState::AnchorGround;
                cell.ground_z = planeZAt(plane, center.x, center.y);
                cell.confidence = 2.0;
                cell.anchor_z_values.push_back(static_cast<float>(cell.ground_z));
                cell.reason = "trail_anchor_plane";
                anchor_cells++;
            }
        }
        res.stats.anchor_cells = anchor_cells;

        std::priority_queue<
            PropagationCandidate,
            std::vector<PropagationCandidate>,
            PropagationCandidateCompare> frontier;
        auto enqueueCandidate = [&](int gx, int gy) {
            if (!propagatedGridInBounds(grid, gx, gy)) {
                return;
            }
            auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
            if (!cell.inside_scope || cell.state != PropagatedCellState::Unknown) {
                return;
            }
            const PropagationEstimate estimate =
                estimateGroundFromNeighbours(grid, gx, gy, params);
            if (!estimate.valid) {
                return;
            }
            frontier.push(PropagationCandidate{estimate.confidence, gx, gy});
        };

        for (int gy = 0; gy < grid.h; ++gy) {
            for (int gx = 0; gx < grid.w; ++gx) {
                const auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
                if (cell.state != PropagatedCellState::AnchorGround) {
                    continue;
                }
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        enqueueCandidate(gx + dx, gy + dy);
                    }
                }
            }
        }

        size_t propagated_ground_cells = 0;
        size_t blocked_unknown_cells = 0;
        size_t measured_obstacle_cells = 0;
        size_t traversable_overhang_cells = 0;

        while (!frontier.empty()) {
            if (abortIfCancelled()) {
                return res;
            }
            const PropagationCandidate candidate = frontier.top();
            frontier.pop();
            auto& cell = grid.cells[propagatedGridIndex(grid, candidate.x, candidate.y)];
            if (cell.state != PropagatedCellState::Unknown) {
                continue;
            }

            const PropagationEstimate estimate =
                estimateGroundFromNeighbours(grid, candidate.x, candidate.y, params);
            if (!estimate.valid) {
                continue;
            }
            const GridClassification classification = classifyPropagatedGridCell(
                cell,
                estimate.predicted_z,
                estimate.confidence,
                params);
            cell.state = classification.state;
            cell.ground_z = classification.ground_z;
            cell.confidence = classification.confidence;
            cell.support_point_count = classification.support_point_count;
            cell.support_base_z = classification.support_base_z;
            cell.support_top_z = classification.support_top_z;
            cell.support_span_z = classification.support_span_z;
            cell.first_non_ground_z = classification.first_non_ground_z;
            cell.clearance_above_ground = classification.clearance_above_ground;
            cell.reason = classification.reason;

            if (classification.state == PropagatedCellState::PropagatedGround) {
                propagated_ground_cells++;
                if (classification.traversable_overhang) {
                    traversable_overhang_cells++;
                }
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        enqueueCandidate(candidate.x + dx, candidate.y + dy);
                    }
                }
            } else if (classification.state == PropagatedCellState::MeasuredObstacle) {
                measured_obstacle_cells++;
            } else if (classification.state == PropagatedCellState::BlockedUnknown) {
                blocked_unknown_cells++;
            }
        }

        size_t sweep_idx = 0;
        for (int gy = 0; gy < grid.h; ++gy) {
            for (int gx = 0; gx < grid.w; ++gx) {
                if ((sweep_idx++ & 0x0FFFu) == 0u && abortIfCancelled()) {
                    return res;
                }
                auto& cell = grid.cells[propagatedGridIndex(grid, gx, gy)];
                if (!cell.inside_scope || cell.state != PropagatedCellState::Unknown) {
                    continue;
                }
                const Point2D center = propagatedGridCellCenter(grid, gx, gy);
                const GridClassification classification = classifyPropagatedGridCell(
                    cell,
                    planeZAt(plane, center.x, center.y),
                    /*predicted_confidence=*/0.25,
                    params);
                cell.state = classification.state;
                cell.ground_z = classification.ground_z;
                cell.confidence = classification.confidence;
                cell.support_point_count = classification.support_point_count;
                cell.support_base_z = classification.support_base_z;
                cell.support_top_z = classification.support_top_z;
                cell.support_span_z = classification.support_span_z;
                cell.first_non_ground_z = classification.first_non_ground_z;
                cell.clearance_above_ground = classification.clearance_above_ground;
                cell.reason = classification.reason;

                if (classification.state == PropagatedCellState::PropagatedGround) {
                    propagated_ground_cells++;
                    if (classification.traversable_overhang) {
                        traversable_overhang_cells++;
                    }
                } else if (classification.state == PropagatedCellState::MeasuredObstacle) {
                    measured_obstacle_cells++;
                } else if (classification.state == PropagatedCellState::BlockedUnknown) {
                    blocked_unknown_cells++;
                }
            }
        }

        size_t ground_points_band = 0;
        for (const auto& cell : grid.cells) {
            if (isGroundCellState(cell.state)) {
                ground_points_band += static_cast<size_t>(std::max(0, cell.point_count));
            }
        }
        res.stats.ground_points_band = ground_points_band;
        res.stats.propagated_ground_cells = propagated_ground_cells;
        res.stats.blocked_unknown_cells = blocked_unknown_cells;
        res.stats.measured_obstacle_cells = measured_obstacle_cells;
        res.stats.traversable_overhang_cells = traversable_overhang_cells;
        res.stats.raw_obstacle_candidates = measured_obstacle_cells + blocked_unknown_cells;
        res.stats.obstacle_points_after_outlier = res.stats.raw_obstacle_candidates;

        int total_holes = 0;
        res.obstacles = polygonizeBlockedCells(
            grid,
            contour_cell_m,
            inflate_radius_m,
            smooth_radius_m,
            geom_smooth_radius_m,
            params.min_contour_area_m2,
            preserve_holes,
            preserve_holes_min_area_m2,
            &total_holes);
        res.stats.total_holes = total_holes;
        res.stats.obstacle_shapes = static_cast<int>(res.obstacles.size());
        std::cout << "[ObstacleDetect] Grid: anchors=" << anchor_cells
                  << ", propagated_ground=" << propagated_ground_cells
                  << ", measured_obstacle=" << measured_obstacle_cells
                  << ", blocked_unknown=" << blocked_unknown_cells
                  << ", traversable_overhang=" << traversable_overhang_cells
                  << ", shapes=" << res.obstacles.size()
                  << ", holes=" << total_holes << "\n";
        res.success = true;
        return res;
    }

    // ------------------------------------------------------------------
    // 4. Obstacle candidate extraction
    // ------------------------------------------------------------------
    obstacle_raw.reset(new PointCloud);
    obstacle_raw->reserve(scoped_cloud->size() / 4);
    size_t ground_band_count = 0;
    size_t candidate_idx = 0;
    for (const auto& pt : scoped_cloud->points) {
        if ((candidate_idx++ & 0x1FFFu) == 0u && abortIfCancelled()) {
            return res;
        }
        PlaneModel active_plane = plane;
        if (params.ground_model_mode == GroundModelMode::LocalHeightField) {
            estimateLocalGroundPlane(
                local_ground, static_cast<double>(pt.x), static_cast<double>(pt.y), &active_plane);
        }
        double sd = signedDist(active_plane, pt);
        bool is_ground = std::abs(sd) <= params.ground_band_m;
        if (is_ground) {
            ground_band_count++;
        }
        bool positive = false;
        bool trough = false;
        if (params.ground_model_mode == GroundModelMode::LocalHeightField) {
            positive = (sd > params.ground_band_m) && (sd <= params.obstacle_z_max);
            trough = (sd < -(params.ground_band_m + params.trough_depth_m));
        } else {
            positive = (sd > params.ground_band_m) && (pt.z <= params.obstacle_z_max);
            trough = (sd < -(params.ground_band_m + params.trough_depth_m)) &&
                     (pt.z <= params.ground_z_max);
        }
        if (positive || trough) {
            obstacle_raw->push_back(pt);
        }
    }
    res.stats.ground_points_band = ground_band_count;
    res.stats.raw_obstacle_candidates = obstacle_raw->size();
    }

    // ------------------------------------------------------------------
    // 5. Statistical outlier removal (PCL SOR)
    // ------------------------------------------------------------------
    // Python-like filter based on mean kNN distance statistics.
    PointCloudPtr obstacle_clean = removeStatisticalOutliersMeanDist(
        obstacle_raw, params.outlier_k, params.outlier_std);
    res.stats.obstacle_points_after_outlier = obstacle_clean ? obstacle_clean->size() : 0;
    if (abortIfCancelled()) {
        return res;
    }
    if (params.detection_method == ObstacleDetectionMethod::ClothSimulationFilter) {
        const size_t denoised_nonground = obstacle_clean ? obstacle_clean->size() : 0;
        res.csf_sor_nonground_cloud = obstacle_clean;
        obstacle_clean = filterCsfNonGroundByClearance(
            obstacle_clean,
            res.csf_ground_cloud,
            params,
            abortIfCancelled);
        if (abortIfCancelled()) {
            return res;
        }
        size_t trail_removed = 0;
        obstacle_clean = removeTrailFootprintObstacleCandidates(
            obstacle_clean,
            path,
            params,
            &trail_removed,
            abortIfCancelled);
        if (abortIfCancelled()) {
            return res;
        }
        std::cout << "[ObstacleDetect] CSF: denoised_nonground=" << denoised_nonground
                  << ", final_obstacle_candidates="
                  << (obstacle_clean ? obstacle_clean->size() : 0)
                  << ", trail_removed=" << trail_removed
                  << ", clearance_min=" << derivedTraversableStepHeight(params)
                  << ", clearance_max=" << params.csf_max_obstacle_clearance_m
                  << "\n";
    }

    // ------------------------------------------------------------------
    // 6. 2D projection + DBSCAN
    // ------------------------------------------------------------------
    std::vector<Point2D> obs_xy;
    obs_xy.reserve(obstacle_clean->size());
    size_t obs_xy_idx = 0;
    for (const auto& pt : obstacle_clean->points) {
        if ((obs_xy_idx++ & 0x1FFFu) == 0u && abortIfCancelled()) {
            return res;
        }
        obs_xy.emplace_back(pt.x, pt.y);
    }
    if (params.detection_method == ObstacleDetectionMethod::ClothSimulationFilter) {
        std::vector<Obstacle2D> clearance_point_cells;
        clearance_point_cells.reserve(obs_xy.size());
        const double point_cell_m = std::max(0.02, 0.5 * grid_cell_m);
        for (const auto& p : obs_xy) {
            Obstacle2D obs;
            obs.outer = rectFromBBox(p.x, p.y, p.x, p.y, point_cell_m, 0.0);
            obs.visual_type = ObstacleVisualType::Unknown;
            clearance_point_cells.push_back(std::move(obs));
        }

        std::vector<Obstacle2D> occupancy_obstacles;
        int total_holes = 0;
        const double csf_grid_cell_m = std::max(0.02, grid_cell_m);
        auto shapes = polygonizeClusterGrid(
            obs_xy,
            csf_grid_cell_m,
            /*contour_cell_m=*/csf_grid_cell_m,
            /*inflate_radius_m=*/0.0,
            /*smooth_radius_m=*/0.0,
            params.min_contour_area_m2);
        occupancy_obstacles.reserve(shapes.size());
        for (auto& sh : shapes) {
            Obstacle2D obs;
            obs.outer = std::move(sh.first);
            obs.holes = std::move(sh.second);
            total_holes += static_cast<int>(obs.holes.size());
            occupancy_obstacles.push_back(std::move(obs));
        }
        res.stats.clusters_found = 0;
        res.stats.groups_merged = static_cast<int>(occupancy_obstacles.size());
        res.stats.total_holes = total_holes;
        res.stats.obstacle_shapes = static_cast<int>(occupancy_obstacles.size());
        res.csf_clearance_point_cells = std::move(clearance_point_cells);
        res.csf_occupancy_obstacles = occupancy_obstacles;
        res.obstacles = std::move(occupancy_obstacles);
        res.success = true;
        std::cout << "[ObstacleDetect] CSF: occupancy-polygonized "
                  << obs_xy.size()
                  << " under-clearance points into "
                  << res.obstacles.size()
                  << " obstacle shape(s) (DBSCAN/hulls/smoothing disabled, cell="
                  << csf_grid_cell_m << ")\n";
        return res;
    }
    std::vector<int> labels = dbscan2D(obs_xy, params.cluster_eps_m, params.cluster_min_pts);
    if (abortIfCancelled()) {
        return res;
    }
    int max_label = -1;
    for (int l : labels) max_label = std::max(max_label, l);
    const int n_clusters = max_label + 1;
    res.stats.clusters_found = n_clusters;

    std::vector<std::vector<Point2D>> clusters;
    clusters.resize(static_cast<size_t>(n_clusters));
    std::vector<Point2D> noise_pts;
    noise_pts.reserve(obs_xy.size());
    for (size_t i = 0; i < obs_xy.size(); ++i) {
        if ((i & 0x1FFFu) == 0u && abortIfCancelled()) {
            return res;
        }
        int l = labels[i];
        if (l < 0) {
            noise_pts.push_back(obs_xy[i]);
            continue;
        }
        clusters[static_cast<size_t>(l)].push_back(obs_xy[i]);
    }

    // Remove empty clusters (shouldn't happen, but keep safe)
    std::vector<std::vector<Point2D>> cluster_list;
    cluster_list.reserve(clusters.size());
    for (auto& c : clusters) {
        if (!c.empty()) cluster_list.push_back(std::move(c));
    }
    clusters.clear();

    // ------------------------------------------------------------------
    // 6b. Preserve micro obstacles (tiny but dense) (matches Python)
    // ------------------------------------------------------------------
    std::vector<Obstacle2D> micro_obstacles;
    if (params.micro_enable && !obs_xy.empty()) {
        std::vector<std::vector<Point2D>> normal_clusters;
        normal_clusters.reserve(cluster_list.size());
        size_t cluster_idx = 0;
        for (auto& cl : cluster_list) {
            if ((cluster_idx++ & 0x3Fu) == 0u && abortIfCancelled()) {
                return res;
            }
            if (isMicroCluster(cl, params)) {
                Obstacle2D obs;
                obs.outer = rectFromBbox(cl, params.micro_min_size_m, params.micro_margin_m);
                micro_obstacles.push_back(std::move(obs));
            } else {
                normal_clusters.push_back(std::move(cl));
            }
        }
        cluster_list = std::move(normal_clusters);

        // Recover micro obstacles from noise points (labels == -1) using tighter DBSCAN
        std::vector<Point2D> noise_pts;
        noise_pts.reserve(obs_xy.size());
        for (size_t i = 0; i < obs_xy.size(); ++i) {
            if (labels[i] == -1) {
                noise_pts.push_back(obs_xy[i]);
            }
        }
        if (static_cast<int>(noise_pts.size()) >= params.micro_min_pts && params.micro_noise_eps_m > 0.0) {
            std::vector<int> micro_labels = dbscan2D(noise_pts, params.micro_noise_eps_m, params.micro_min_pts);
            if (abortIfCancelled()) {
                return res;
            }
            int micro_max_label = -1;
            for (int l : micro_labels) micro_max_label = std::max(micro_max_label, l);
            const int micro_n_clusters = micro_max_label + 1;
            if (micro_n_clusters > 0) {
                std::vector<std::vector<Point2D>> micro_clusters(static_cast<size_t>(micro_n_clusters));
                for (size_t i = 0; i < noise_pts.size(); ++i) {
                    int l = micro_labels[i];
                    if (l < 0) continue;
                    micro_clusters[static_cast<size_t>(l)].push_back(noise_pts[i]);
                }
                for (auto& mc : micro_clusters) {
                    if (mc.empty()) continue;
                    if (!isMicroCluster(mc, params)) continue;
                    Obstacle2D obs;
                    obs.outer = rectFromBbox(mc, params.micro_min_size_m, params.micro_margin_m);
                    micro_obstacles.push_back(std::move(obs));
                }
            }
        }
    }

    if (cluster_list.empty()) {
        // Only micro obstacles exist (or no obstacles at all)
        res.stats.total_holes = 0;
        res.stats.obstacle_shapes = static_cast<int>(micro_obstacles.size());
        res.obstacles = std::move(micro_obstacles);
        res.success = true;
        return res;
    }

    // ------------------------------------------------------------------
    // 7. Merge nearby clusters (union-find) + polygonize groups (AUTO)
    // ------------------------------------------------------------------
    const double merge_d2 = params.merge_distance_m * params.merge_distance_m;

    struct ClusterKD {
        std::vector<Point2D> pts;
        Point2D minp{0, 0};
        Point2D maxp{0, 0};
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
        std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> tree;
    };

    std::vector<ClusterKD> ck;
    ck.reserve(cluster_list.size());
    for (auto& c : cluster_list) {
        ClusterKD entry;
        entry.pts = std::move(c);
        entry.minp = entry.maxp = entry.pts.front();
        for (const auto& p : entry.pts) {
            entry.minp.x = std::min(entry.minp.x, p.x);
            entry.minp.y = std::min(entry.minp.y, p.y);
            entry.maxp.x = std::max(entry.maxp.x, p.x);
            entry.maxp.y = std::max(entry.maxp.y, p.y);
        }
        entry.pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        entry.pcl_cloud->reserve(entry.pts.size());
        for (const auto& p : entry.pts) {
            entry.pcl_cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x), static_cast<float>(p.y), 0.0f));
        }
        entry.tree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        entry.tree->setInputCloud(entry.pcl_cloud);
        ck.push_back(std::move(entry));
    }

    struct UnionFind {
        std::vector<int> parent;
        explicit UnionFind(int n) : parent(static_cast<size_t>(n)) {
            for (int i = 0; i < n; ++i) parent[static_cast<size_t>(i)] = i;
        }
        int find(int x) {
            int r = x;
            while (parent[static_cast<size_t>(r)] != r) {
                r = parent[static_cast<size_t>(r)];
            }
            while (parent[static_cast<size_t>(x)] != x) {
                int px = parent[static_cast<size_t>(x)];
                parent[static_cast<size_t>(x)] = r;
                x = px;
            }
            return r;
        }
        void unite(int a, int b) {
            int ra = find(a);
            int rb = find(b);
            if (ra != rb) parent[static_cast<size_t>(ra)] = rb;
        }
    };

    UnionFind uf(static_cast<int>(ck.size()));

    auto aabbMinDist2 = [](const ClusterKD& a, const ClusterKD& b) -> double {
        double dx = 0.0;
        if (a.maxp.x < b.minp.x) dx = b.minp.x - a.maxp.x;
        else if (b.maxp.x < a.minp.x) dx = a.minp.x - b.maxp.x;
        double dy = 0.0;
        if (a.maxp.y < b.minp.y) dy = b.minp.y - a.maxp.y;
        else if (b.maxp.y < a.minp.y) dy = a.minp.y - b.maxp.y;
        return dx * dx + dy * dy;
    };

    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist2(1);

    for (int i = 0; i < static_cast<int>(ck.size()); ++i) {
        if ((i & 0x0Fu) == 0 && abortIfCancelled()) {
            return res;
        }
        for (int j = i + 1; j < static_cast<int>(ck.size()); ++j) {
            if (aabbMinDist2(ck[static_cast<size_t>(i)], ck[static_cast<size_t>(j)]) > merge_d2) {
                continue;
            }
            double best_d2 = std::numeric_limits<double>::infinity();
            for (const auto& p : ck[static_cast<size_t>(i)].pcl_cloud->points) {
                int found = ck[static_cast<size_t>(j)].tree->nearestKSearch(p, 1, nn_idx, nn_dist2);
                if (found > 0) {
                    best_d2 = std::min(best_d2, static_cast<double>(nn_dist2[0]));
                    if (best_d2 <= merge_d2) break;
                }
            }
            if (best_d2 <= merge_d2) {
                uf.unite(i, j);
            }
        }
    }

    std::unordered_map<int, std::vector<int>> groups;
    groups.reserve(ck.size());
    for (int i = 0; i < static_cast<int>(ck.size()); ++i) {
        groups[uf.find(i)].push_back(i);
    }
    res.stats.groups_merged = static_cast<int>(groups.size());

    std::vector<Obstacle2D> obstacles_out;
    int total_holes = 0;

    for (const auto& kv : groups) {
        if (abortIfCancelled()) {
            return res;
        }
        const auto& idxs = kv.second;
        std::vector<Point2D> merged_pts;
        size_t total_pts = 0;
        for (int ci : idxs) total_pts += ck[static_cast<size_t>(ci)].pts.size();
        merged_pts.reserve(total_pts);
        for (int ci : idxs) {
            const auto& pts = ck[static_cast<size_t>(ci)].pts;
            merged_pts.insert(merged_pts.end(), pts.begin(), pts.end());
        }
        if (merged_pts.empty()) continue;

        // Representative hull (used for AUTO heuristics and hull fallback)
        Polygon2D merged_hull;
        try {
            if (merged_pts.size() >= 3) {
                merged_hull = computeConvexHull(merged_pts);
            } else {
                merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
            }
        } catch (...) {
            merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
        }
        double hull_area = polygonArea(merged_hull);
        if (hull_area <= 1e-12) {
            merged_hull = rectFromBbox(merged_pts, 0.05, 0.0);
            hull_area = polygonArea(merged_hull);
        }

        bool use_grid = false;
        bool allow_grid = (params.polygon_mode == ObstaclePolygonMode::Auto ||
                           params.polygon_mode == ObstaclePolygonMode::Grid);

        if (allow_grid) {
            const bool prefer_grid = (params.polygon_mode == ObstaclePolygonMode::Auto) &&
                (geom_smooth_radius_m > 0.0 || contour_cell_m > grid_cell_m + 1e-12);

            // Hollow ratio computed from *non-inflated* occupancy (padding=0.20)
            bool hollow_trigger = false;
            if (grid_cell_m > 0.0 && hull_area > params.min_contour_area_m2) {
                OccGrid occ0 = occupancyFromPoints(merged_pts, grid_cell_m, 0.20);
                size_t occ_sum = 0;
                for (uint8_t v : occ0.occ) occ_sum += (v != 0);
                const double occ_area = static_cast<double>(occ_sum) * (grid_cell_m * grid_cell_m);
                const double hollow_ratio = occ_area / std::max(hull_area, 1e-9);
                hollow_trigger = (hollow_ratio < params.hollow_ratio_thresh);
            }

            use_grid = (params.polygon_mode == ObstaclePolygonMode::Grid) || prefer_grid || hollow_trigger;
        }

        if (use_grid) {
            // Keep polygonization simple; apply smoothing once as a post-process (matches Python).
            auto shapes = polygonizeClusterGrid(
                merged_pts,
                grid_cell_m,
                contour_cell_m,
                inflate_radius_m,
                /*smooth_radius_m=*/0.0,
                params.min_contour_area_m2);

            if (!shapes.empty()) {
                auto pointInAnyShape = [&](const Point2D& p) -> bool {
                    for (const auto& sh : shapes) {
                        if (!pointInPolyRayCast(p, sh.first)) continue;
                        bool in_hole = false;
                        for (const auto& h : sh.second) {
                            if (pointInPolyRayCast(p, h)) {
                                in_hole = true;
                                break;
                            }
                        }
                        if (!in_hole) return true;
                    }
                    return false;
                };

                // Preserve tiny/disconnected source clusters that can be dropped by contour cleanup.
                for (int ci : idxs) {
                    const auto& cpts = ck[static_cast<size_t>(ci)].pts;
                    if (cpts.empty()) continue;
                    bool covered = false;
                    const size_t stride = std::max<size_t>(1, cpts.size() / 12);
                    for (size_t k = 0; k < cpts.size(); k += stride) {
                        if (pointInAnyShape(cpts[k])) {
                            covered = true;
                            break;
                        }
                    }
                    if (!covered && pointInAnyShape(cpts.back())) {
                        covered = true;
                    }
                    if (covered) continue;

                    Polygon2D tiny = rectFromBbox(cpts, std::max(0.02, 0.75 * grid_cell_m), 0.0);
                    shapes.emplace_back(std::move(tiny), std::vector<Polygon2D>{});
                }

                for (auto& sh : shapes) {
                    Obstacle2D obs;
                    obs.outer = std::move(sh.first);
                    obs.holes = std::move(sh.second);
                    total_holes += static_cast<int>(obs.holes.size());
                    obstacles_out.push_back(std::move(obs));
                }
                continue;
            }
            // If grid fails, fall back to hull.
        }

        Obstacle2D obs;
        obs.outer = std::move(merged_hull);
        obstacles_out.push_back(std::move(obs));
    }

    // 7c. Optional geometric smoothing (rolling-disk closing) on final polygons (grid fallback).
    if (geom_smooth_radius_m > 0.0 && !obstacles_out.empty()) {
        obstacles_out = smoothShapesRollingDiskGrid(
            obstacles_out,
            geom_smooth_radius_m,
            grid_cell_m,
            contour_cell_m,
            params.min_contour_area_m2,
            preserve_holes,
            preserve_holes_min_area_m2);
    }
    if (abortIfCancelled()) {
        return res;
    }

    // Append preserved micro obstacles unchanged (Python skips smoothing for micro shapes).
    if (!micro_obstacles.empty()) {
        obstacles_out.reserve(obstacles_out.size() + micro_obstacles.size());
        for (auto& m : micro_obstacles) {
            obstacles_out.push_back(std::move(m));
        }
    }

    int final_holes = 0;
    for (const auto& o : obstacles_out) {
        final_holes += static_cast<int>(o.holes.size());
    }
    res.stats.total_holes = final_holes;
    res.stats.obstacle_shapes = static_cast<int>(obstacles_out.size());
    res.obstacles = std::move(obstacles_out);
    res.success = true;
    return res;
}

ObstacleDetectionResult detectObstacles(
    const PointCloudPtr& cloud,
    const std::vector<PathState>& driven_path,
    const Polygon2D* roi_or_boundary,
    const ObstacleDetectionParams& params) {
    if (params.detection_method == ObstacleDetectionMethod::PatchworkRawBundle) {
        (void)driven_path;
        return detectObstaclesPatchworkBundleImpl(cloud, roi_or_boundary, params);
    }
    return detectObstaclesAuto(cloud, driven_path, roi_or_boundary, params);
}

}  // namespace f2c_cpp

