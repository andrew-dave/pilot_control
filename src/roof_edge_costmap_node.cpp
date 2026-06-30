// Roof-edge costmap node (F1 of the autonomy CONOPS).
//
// Builds a persistent, map-frame 2.5D elevation grid from the tilt-levelled
// Fast-LIO cloud, classifies every confident cell as FREE / LETHAL / UNKNOWN by
// the local step it forms against its neighbours (positive obstacles AND
// negative drops/cliffs), and publishes a 3-value OccupancyGrid costmap, a
// height-map point cloud, and a continuously replanned JPS path from the robot
// to an operator-supplied /goal_pose.
//
// v1 is PASSIVE: nothing here commands the robot. It is a visualisation /
// integration surface so the costmap + replanned path can be watched in RViz
// while the operator teleops. MPC authority is a later phase.
//
// Levelling reuses the Option-B scheme from local_nav_grid_publisher: the
// robot-frame transform is derived live by matching a raw /Odometry sample to
// the /Odometry_tilt_corrected_diff sample of the same stamp. The NPZ tilt
// calibration is a STRICT arming gate (no fixed-pitch fallback): if the file is
// missing or unparseable the node refuses to process clouds.
//
// See docs/AUTONOMY_CONOPS.md (F1) for the full specification and rationale.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "roof_edge_grid_planner.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kRawOdomBufferMaxSamples = 200;

uint64_t stampToNs(int32_t sec, uint32_t nanosec) {
    return (static_cast<uint64_t>(sec) * 1000000000ULL) + static_cast<uint64_t>(nanosec);
}

Eigen::Matrix3d quaternionToMatrix(double x, double y, double z, double w) {
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    Eigen::Matrix3d rotation;
    rotation << 1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
                2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
                2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy);
    return rotation;
}

double extractYawFromMatrix(const Eigen::Matrix3d& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

// Convert a rotation matrix to a (x, y, z, w) quaternion (Shepperd's method),
// used to publish the static camera_init -> robot_init TF.
void matrixToQuaternion(const Eigen::Matrix3d& r, double& qx, double& qy, double& qz,
                        double& qw) {
    const double trace = r(0, 0) + r(1, 1) + r(2, 2);
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (r(2, 1) - r(1, 2)) / s;
        qy = (r(0, 2) - r(2, 0)) / s;
        qz = (r(1, 0) - r(0, 1)) / s;
    } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
        double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
        qw = (r(2, 1) - r(1, 2)) / s;
        qx = 0.25 * s;
        qy = (r(0, 1) + r(1, 0)) / s;
        qz = (r(0, 2) + r(2, 0)) / s;
    } else if (r(1, 1) > r(2, 2)) {
        double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
        qw = (r(0, 2) - r(2, 0)) / s;
        qx = (r(0, 1) + r(1, 0)) / s;
        qy = 0.25 * s;
        qz = (r(1, 2) + r(2, 1)) / s;
    } else {
        double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
        qw = (r(1, 0) - r(0, 1)) / s;
        qx = (r(0, 2) + r(2, 0)) / s;
        qy = (r(1, 2) + r(2, 1)) / s;
        qz = 0.25 * s;
    }
}

std::string shellEscapeSingleQuoted(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size() + 8);
    for (char ch : text) {
        if (ch == '\'') {
            escaped += "'\\''";
        } else {
            escaped.push_back(ch);
        }
    }
    return escaped;
}

std::string makeTempPath(const std::string& stem) {
    const auto now_ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    return (std::filesystem::temp_directory_path() /
            (stem + "_" + std::to_string(now_ticks) + ".txt"))
        .string();
}

bool loadMatrixFromText(const std::string& path, Eigen::Matrix3d& matrix) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            file >> matrix(row, col);
            if (!file.good()) return false;
        }
    }
    return true;
}

// Validate the static tilt calibration NPZ by extracting R_map. This is purely
// an arming gate: the live transform is derived from odometry matching, but a
// present + parseable calibration proves odom_tilt_corrector is genuinely
// calibrated (the two share the same file), so the levelled frame is trustworthy.
bool validateCalibration(const std::string& npz_file, const rclcpp::Logger& logger) {
    if (npz_file.empty()) return false;
    if (!std::filesystem::exists(npz_file)) {
        RCLCPP_ERROR(logger, "Tilt calibration file does not exist: %s", npz_file.c_str());
        return false;
    }
    const std::string r_map_file = makeTempPath("roof_edge_r_map");
    const std::string cmd =
        "python3 -c \"import numpy as np; "
        "d=np.load('" + shellEscapeSingleQuoted(npz_file) + "'); "
        "np.savetxt('" + shellEscapeSingleQuoted(r_map_file) + "', d['R_map'])\" "
        ">/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    bool ok = false;
    if (rc == 0) {
        Eigen::Matrix3d r_map;
        ok = loadMatrixFromText(r_map_file, r_map);
    }
    std::filesystem::remove(r_map_file);
    if (!ok) {
        RCLCPP_ERROR(logger, "Failed to parse R_map from calibration %s", npz_file.c_str());
    }
    return ok;
}

struct RawOdomSample {
    uint64_t stamp_ns = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
};

// Persistent, map-frame 2.5D elevation grid. Grows as the robot explores. Each
// cell carries Welford running mean/variance of point height (representative
// ground height + confidence), the observed z extremes, and two driven masks
// (full footprint vs wheel-track) for the empirical traversability prior.
class PersistentHeightMap {
public:
    PersistentHeightMap(double resolution, double grow_pad_m, long long max_cells,
                        double overhead_margin_m)
        : res_(resolution),
          grow_pad_cells_(std::max(1, static_cast<int>(std::ceil(grow_pad_m / resolution)))),
          max_cells_(max_cells),
          overhead_margin_(overhead_margin_m) {}

    double resolution() const { return res_; }
    int width() const { return w_; }
    int height() const { return h_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }
    bool empty() const { return w_ == 0 || h_ == 0; }

    bool worldToCell(double x, double y, int& gx, int& gy) const {
        gx = static_cast<int>(std::floor((x - origin_x_) / res_));
        gy = static_cast<int>(std::floor((y - origin_y_) / res_));
        return gx >= 0 && gy >= 0 && gx < w_ && gy < h_;
    }

    int clampCellX(double x) const {
        return std::clamp(static_cast<int>(std::floor((x - origin_x_) / res_)), 0,
                          std::max(0, w_ - 1));
    }
    int clampCellY(double y) const {
        return std::clamp(static_cast<int>(std::floor((y - origin_y_) / res_)), 0,
                          std::max(0, h_ - 1));
    }
    double cellCenterX(int gx) const { return origin_x_ + (gx + 0.5) * res_; }
    double cellCenterY(int gy) const { return origin_y_ + (gy + 0.5) * res_; }

    // Grow the grid (with padding) so the world rect fits. Returns false if the
    // required allocation would exceed the cell budget (caller skips the region).
    bool ensure(double minx, double miny, double maxx, double maxy) {
        if (w_ == 0) {
            origin_x_ = std::floor(minx / res_) * res_ - grow_pad_cells_ * res_;
            origin_y_ = std::floor(miny / res_) * res_ - grow_pad_cells_ * res_;
            w_ = static_cast<int>(std::ceil((maxx - origin_x_) / res_)) + grow_pad_cells_;
            h_ = static_cast<int>(std::ceil((maxy - origin_y_) / res_)) + grow_pad_cells_;
            if (static_cast<long long>(w_) * h_ > max_cells_) {
                w_ = h_ = 0;
                return false;
            }
            allocate(w_, h_);
            return true;
        }

        const double cur_maxx = origin_x_ + w_ * res_;
        const double cur_maxy = origin_y_ + h_ * res_;
        if (minx >= origin_x_ && miny >= origin_y_ && maxx < cur_maxx && maxy < cur_maxy) {
            return true;  // already covered
        }

        double new_minx = std::min(origin_x_, minx - grow_pad_cells_ * res_);
        double new_miny = std::min(origin_y_, miny - grow_pad_cells_ * res_);
        double new_maxx = std::max(cur_maxx, maxx + grow_pad_cells_ * res_);
        double new_maxy = std::max(cur_maxy, maxy + grow_pad_cells_ * res_);

        const double new_origin_x = std::floor(new_minx / res_) * res_;
        const double new_origin_y = std::floor(new_miny / res_) * res_;
        const int new_w = static_cast<int>(std::ceil((new_maxx - new_origin_x) / res_));
        const int new_h = static_cast<int>(std::ceil((new_maxy - new_origin_y) / res_));
        if (static_cast<long long>(new_w) * new_h > max_cells_) {
            return false;
        }

        const int off_x = static_cast<int>(std::llround((origin_x_ - new_origin_x) / res_));
        const int off_y = static_cast<int>(std::llround((origin_y_ - new_origin_y) / res_));

        PersistentHeightMap grown(res_, 0.0, max_cells_, overhead_margin_);
        grown.grow_pad_cells_ = grow_pad_cells_;
        grown.origin_x_ = new_origin_x;
        grown.origin_y_ = new_origin_y;
        grown.w_ = new_w;
        grown.h_ = new_h;
        grown.allocate(new_w, new_h);

        for (int y = 0; y < h_; ++y) {
            const int dst_row = (y + off_y) * new_w + off_x;
            const int src_row = y * w_;
            for (int x = 0; x < w_; ++x) {
                const int s = src_row + x;
                if (n_[s] == 0 && drvF_[s] == 0 && drvW_[s] == 0) continue;
                const int d = dst_row + x;
                grown.n_[d] = n_[s];
                grown.mu_[d] = mu_[s];
                grown.m2_[d] = m2_[s];
                grown.zmin_[d] = zmin_[s];
                grown.zmax_[d] = zmax_[s];
                grown.drvF_[d] = drvF_[s];
                grown.drvW_[d] = drvW_[s];
            }
        }
        *this = std::move(grown);
        return true;
    }

    void addPoint(double x, double y, double z) {
        int gx, gy;
        if (!worldToCell(x, y, gx, gy)) return;
        const int i = gy * w_ + gx;
        if (n_[i] == 0) {
            n_[i] = 1;
            mu_[i] = static_cast<float>(z);
            m2_[i] = 0.0f;
            zmin_[i] = static_cast<float>(z);
            zmax_[i] = static_cast<float>(z);
            return;
        }
        // Terrain-relative overhead reject: drop returns more than
        // overhead_margin_ above the column's established ground (its running
        // min). On curved/sloped roofs the reference rides with the local
        // surface, so soffits / pipes / ceilings above the robot never register
        // as obstacles, while a real obstacle's lower band (floor → margin) is
        // still integrated and trips the LETHAL test. Excluding overhead points
        // also keeps their high z out of the mean/variance, so floor cells under
        // an overhang stay ground-confident → FREE (drive-under).
        if (overhead_margin_ > 0.0 &&
            (z - static_cast<double>(zmin_[i])) > overhead_margin_) {
            return;
        }
        n_[i] += 1;
        const double delta = z - mu_[i];
        mu_[i] += static_cast<float>(delta / n_[i]);
        m2_[i] += static_cast<float>(delta * (z - mu_[i]));
        zmin_[i] = std::min(zmin_[i], static_cast<float>(z));
        zmax_[i] = std::max(zmax_[i], static_cast<float>(z));
    }

    // Stamp a rotated rectangle (robot footprint or a wheel strip) into the
    // driven masks. Caller must ensure() the AABB beforehand.
    void stampRect(double cx, double cy, double yaw, double half_len, double half_wid,
                   bool footprint, bool wheel) {
        if (w_ == 0) return;
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        const double diag = std::hypot(half_len, half_wid);
        const int gx0 = clampCellX(cx - diag), gx1 = clampCellX(cx + diag);
        const int gy0 = clampCellY(cy - diag), gy1 = clampCellY(cy + diag);
        for (int gy = gy0; gy <= gy1; ++gy) {
            for (int gx = gx0; gx <= gx1; ++gx) {
                const double dx = cellCenterX(gx) - cx;
                const double dy = cellCenterY(gy) - cy;
                const double lx = cyaw * dx + syaw * dy;
                const double ly = -syaw * dx + cyaw * dy;
                if (std::abs(lx) > half_len || std::abs(ly) > half_wid) continue;
                const int i = gy * w_ + gx;
                if (footprint) drvF_[i] = 1;
                if (wheel) drvW_[i] = 1;
            }
        }
    }

    // Per-cell accessors used by the classifier (global cell index).
    uint32_t count(int i) const { return n_[i]; }
    float mean(int i) const { return mu_[i]; }
    float m2(int i) const { return m2_[i]; }
    float zMin(int i) const { return zmin_[i]; }
    float zMax(int i) const { return zmax_[i]; }
    bool drivenFootprint(int i) const { return drvF_[i] != 0; }
    bool drivenWheel(int i) const { return drvW_[i] != 0; }
    int index(int gx, int gy) const { return gy * w_ + gx; }

private:
    void allocate(int w, int h) {
        const size_t n = static_cast<size_t>(w) * h;
        n_.assign(n, 0);
        mu_.assign(n, 0.0f);
        m2_.assign(n, 0.0f);
        zmin_.assign(n, 0.0f);
        zmax_.assign(n, 0.0f);
        drvF_.assign(n, 0);
        drvW_.assign(n, 0);
    }

    double res_ = 0.05;
    int grow_pad_cells_ = 100;
    long long max_cells_ = 6'000'000;
    double overhead_margin_ = 0.40;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    int w_ = 0;
    int h_ = 0;
    std::vector<uint32_t> n_;
    std::vector<float> mu_;
    std::vector<float> m2_;
    std::vector<float> zmin_;
    std::vector<float> zmax_;
    std::vector<uint8_t> drvF_;
    std::vector<uint8_t> drvW_;
};

}  // namespace

class RoofEdgeCostmapNode : public rclcpp::Node {
public:
    RoofEdgeCostmapNode() : Node("roof_edge_costmap") {
        cloud_topic_ = declareString("cloud_topic", "/cloud_registered");
        raw_odometry_topic_ = declareString("raw_odometry_topic", "/Odometry");
        corrected_odometry_topic_ =
            declareString("corrected_odometry_topic", "/Odometry_tilt_corrected_diff");
        goal_topic_ = declareString("goal_topic", "/goal_pose");
        costmap_topic_ = declareString("costmap_topic", "/roof_edge/costmap");
        heightmap_topic_ = declareString("heightmap_topic", "/roof_edge/heightmap");
        path_topic_ = declareString("path_topic", "/roof_edge/planned_path");
        calibration_file_ = declareString("calibration_file", "");
        lidar_world_frame_ = declareString("lidar_world_frame", "camera_init");

        cell_size_m_ = declareDouble("cell_size_m", 0.05);
        step_up_m_ = declareDouble("step_up_m", 0.02);
        step_down_m_ = declareDouble("step_down_m", 0.04);
        driven_override_max_step_m_ = declareDouble("driven_override_max_step_m", 0.04);
        min_obs_count_ = static_cast<int>(declareInt("min_obs_count", 5));
        max_stderr_m_ = declareDouble("max_stderr_m", 0.02);

        // Eager (variance-independent) obstacle/drop evidence. Tall structures
        // (lockers, walls) create high within-cell z-variance and would never
        // pass the low-variance ground gate, so they are detected directly from
        // the z extremes (zmax/zmin) relative to the surrounding floor, gated by
        // a point-count so single flyers can't trip them.
        obstacle_min_points_ = static_cast<int>(declareInt("obstacle_min_points", 4));
        obstacle_height_m_ = declareDouble("obstacle_height_m", 0.04);
        obstacle_corroborate_rise_m_ = declareDouble("obstacle_corroborate_rise_m", 0.02);
        big_drop_m_ = declareDouble("big_drop_m", 0.20);
        drop_corroborate_m_ = declareDouble("drop_corroborate_m", 0.05);
        local_floor_radius_cells_ =
            std::max(1, static_cast<int>(declareInt("local_floor_radius_cells", 6)));

        robot_length_m_ = declareDouble("robot_length_m", 0.48);
        robot_width_m_ = declareDouble("robot_width_m", 0.45);
        track_width_m_ = declareDouble("track_width_m", 0.38);
        wheel_width_m_ = declareDouble("wheel_width_m", 0.07);
        inflation_radius_m_ = declareDouble("inflation_radius_m", 0.35);

        window_radius_m_ = declareDouble("window_radius_m", 8.0);
        max_plan_radius_m_ = declareDouble("max_plan_radius_m", 25.0);
        region_pad_m_ = declareDouble("region_pad_m", 2.0);
        max_range_m_ = declareDouble("max_range_m", 30.0);
        z_min_clip_m_ = declareDouble("z_min_clip_m", -15.0);
        z_max_clip_m_ = declareDouble("z_max_clip_m", 2.0);
        // Terrain-relative overhead reject (per-column, see addPoint). The loose
        // absolute z_max_clip above only trims wild flyers; this is the precise
        // "ignore structure above the robot" gate, measured against local ground
        // so it works on curved/peaked roofs.
        overhead_margin_m_ = declareDouble("overhead_margin_m", 0.40);

        publish_rate_hz_ = declareDouble("publish_rate_hz", 2.0);
        heightmap_rate_hz_ = declareDouble("heightmap_rate_hz", 1.0);
        heightmap_decimate_ = std::max(1, static_cast<int>(declareInt("heightmap_decimate", 2)));
        cloud_decimate_ = std::max(1, static_cast<int>(declareInt("cloud_decimate", 1)));
        publish_heightmap_ = declareBool("publish_heightmap", true);
        plan_through_unknown_ = declareBool("plan_through_unknown", false);
        grow_pad_m_ = declareDouble("grow_pad_m", 5.0);
        max_map_cells_ = static_cast<long long>(declareInt("max_map_cells", 6'000'000));
        driven_stamp_min_move_m_ = declareDouble("driven_stamp_min_move_m", 0.03);
        snap_radius_m_ = declareDouble("snap_radius_m", 0.5);

        // STRICT arming gate: no fixed-pitch fallback. A missing or unparseable
        // calibration means the levelled frame can't be trusted, so the node
        // stays disarmed and ignores clouds rather than emit a wrong costmap.
        armed_ = validateCalibration(calibration_file_, this->get_logger());
        if (!armed_) {
            RCLCPP_FATAL(this->get_logger(),
                         "roof_edge_costmap DISARMED: tilt calibration invalid (%s). "
                         "No costmap will be produced. Pass a valid calibration_file.",
                         calibration_file_.c_str());
        }

        map_ = std::make_unique<PersistentHeightMap>(cell_size_m_, grow_pad_m_, max_map_cells_,
                                                     overhead_margin_m_);
        tf_static_broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        rclcpp::QoS cloud_qos(rclcpp::KeepLast(10));
        cloud_qos.reliable();
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, cloud_qos,
            std::bind(&RoofEdgeCostmapNode::onCloud, this, std::placeholders::_1));
        raw_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            raw_odometry_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
            std::bind(&RoofEdgeCostmapNode::onRawOdom, this, std::placeholders::_1));
        corrected_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            corrected_odometry_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
            std::bind(&RoofEdgeCostmapNode::onCorrectedOdom, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, rclcpp::QoS(rclcpp::KeepLast(5)),
            std::bind(&RoofEdgeCostmapNode::onGoal, this, std::placeholders::_1));

        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        if (publish_heightmap_) {
            heightmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                heightmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        }

        const double rate = (publish_rate_hz_ > 0.0) ? publish_rate_hz_ : 2.0;
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate)),
            std::bind(&RoofEdgeCostmapNode::onPublishTick, this));

        RCLCPP_INFO(this->get_logger(), "roof_edge_costmap started (armed=%s)",
                    armed_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  cloud:     %s", cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  costmap:   %s", costmap_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  heightmap: %s", heightmap_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  path:      %s", path_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  goal in:   %s", goal_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "  cell=%.3fm step_up=%.3fm step_down=%.3fm inflation=%.3fm",
                    cell_size_m_, step_up_m_, step_down_m_, inflation_radius_m_);
        RCLCPP_INFO(this->get_logger(),
                    "  obstacle_height=%.3fm overhead_margin=%.3fm (terrain-relative)",
                    obstacle_height_m_, overhead_margin_m_);
    }

private:
    std::string declareString(const std::string& name, const std::string& def) {
        this->declare_parameter<std::string>(name, def);
        return this->get_parameter(name).as_string();
    }
    double declareDouble(const std::string& name, double def) {
        this->declare_parameter<double>(name, def);
        return this->get_parameter(name).as_double();
    }
    int64_t declareInt(const std::string& name, int64_t def) {
        this->declare_parameter<int64_t>(name, def);
        return this->get_parameter(name).as_int();
    }
    bool declareBool(const std::string& name, bool def) {
        this->declare_parameter<bool>(name, def);
        return this->get_parameter(name).as_bool();
    }

    // --- Option-B levelling: derive the robot-frame transform live by matching
    // a raw odom sample to the corrected odom sample of the same timestamp. ---

    void onRawOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg || !armed_ || transform_initialized_) return;
        RawOdomSample sample;
        sample.stamp_ns = stampToNs(msg->header.stamp.sec, msg->header.stamp.nanosec);
        sample.position = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z};
        sample.orientation = quaternionToMatrix(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        raw_odom_buffer_.push_back(std::move(sample));
        while (raw_odom_buffer_.size() > kRawOdomBufferMaxSamples) {
            raw_odom_buffer_.pop_front();
        }
        if (pending_corrected_valid_) tryInitializeTransform();
    }

    void onCorrectedOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg || !armed_) return;
        if (corrected_frame_id_.empty() && !msg->header.frame_id.empty()) {
            corrected_frame_id_ = msg->header.frame_id;
        }
        latest_robot_position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                                  msg->pose.pose.position.z};
        const Eigen::Matrix3d r = quaternionToMatrix(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        latest_robot_yaw_ = extractYawFromMatrix(r);
        odom_ready_ = true;

        if (!transform_initialized_) {
            pending_corrected_stamp_ns_ =
                stampToNs(msg->header.stamp.sec, msg->header.stamp.nanosec);
            pending_corrected_position_ = latest_robot_position_;
            pending_corrected_orientation_ = r;
            pending_corrected_valid_ = true;
            tryInitializeTransform();
            return;
        }
        stampDrivenFootprint();
    }

    void tryInitializeTransform() {
        if (transform_initialized_ || !pending_corrected_valid_) return;
        const auto it = std::find_if(raw_odom_buffer_.begin(), raw_odom_buffer_.end(),
                                     [this](const RawOdomSample& s) {
                                         return s.stamp_ns == pending_corrected_stamp_ns_;
                                     });
        if (it == raw_odom_buffer_.end()) return;

        r_init_ = pending_corrected_orientation_ * it->orientation.transpose();
        p0_lidar_ = it->position - (r_init_.transpose() * pending_corrected_position_);
        transform_initialized_ = true;
        pending_corrected_valid_ = false;
        raw_odom_buffer_.clear();
        RCLCPP_INFO(this->get_logger(),
                    "Levelling transform initialised (frame=%s, yaw=%.2f deg)",
                    corrected_frame_id_.c_str(), extractYawFromMatrix(r_init_) * 180.0 / kPi);
        broadcastLevelledFrameTf();
    }

    // Publish the static lidar_world_frame (camera_init) -> robot_init transform
    // so the raw /cloud_registered (camera_init) and the levelled costmap /
    // height-map (robot_init) co-register in RViz without an external identity
    // static_transform_publisher hack. A point maps as
    //   p_robot = r_init_ * (p_cam - p0_lidar_),
    // so the pose of robot_init expressed in camera_init (what tf wants for
    // parent=camera_init, child=robot_init) is rotation r_init_^T, translation
    // p0_lidar_.
    void broadcastLevelledFrameTf() {
        if (!tf_static_broadcaster_) return;
        const Eigen::Matrix3d r_cam_robot = r_init_.transpose();
        double qx, qy, qz, qw;
        matrixToQuaternion(r_cam_robot, qx, qy, qz, qw);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = lidar_world_frame_;
        tf.child_frame_id = frameId();
        tf.transform.translation.x = p0_lidar_.x();
        tf.transform.translation.y = p0_lidar_.y();
        tf.transform.translation.z = p0_lidar_.z();
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;
        tf_static_broadcaster_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(), "Broadcast static TF %s -> %s",
                    lidar_world_frame_.c_str(), frameId().c_str());
    }

    bool transformPoint(double rx, double ry, double rz, double& cx, double& cy,
                        double& cz) const {
        const double ex = rx - p0_lidar_.x();
        const double ey = ry - p0_lidar_.y();
        const double ez = rz - p0_lidar_.z();
        cx = r_init_(0, 0) * ex + r_init_(0, 1) * ey + r_init_(0, 2) * ez;
        cy = r_init_(1, 0) * ex + r_init_(1, 1) * ey + r_init_(1, 2) * ez;
        cz = r_init_(2, 0) * ex + r_init_(2, 1) * ey + r_init_(2, 2) * ez;
        return std::isfinite(cx) && std::isfinite(cy) && std::isfinite(cz);
    }

    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg) return;
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        have_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal: [%.2f, %.2f]", goal_x_, goal_y_);
    }

    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg || !armed_ || !transform_initialized_ || msg->data.empty() ||
            msg->point_step == 0) {
            return;
        }

        const double rx = latest_robot_position_.x();
        const double ry = latest_robot_position_.y();
        const double max_range_sq = max_range_m_ * max_range_m_;

        // Levelled points for this scan + their XY bbox, so the persistent grid
        // is expanded once before integrating (no per-point reallocation).
        std::vector<Eigen::Vector3f> pts;
        pts.reserve(msg->data.size() / msg->point_step);
        double minx = std::numeric_limits<double>::max();
        double miny = std::numeric_limits<double>::max();
        double maxx = std::numeric_limits<double>::lowest();
        double maxy = std::numeric_limits<double>::lowest();

        try {
            sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
            int stride = 0;
            for (; ix != ix.end(); ++ix, ++iy, ++iz, ++stride) {
                if (cloud_decimate_ > 1 && (stride % cloud_decimate_) != 0) continue;
                const double rxp = *ix, ryp = *iy, rzp = *iz;
                if (!std::isfinite(rxp) || !std::isfinite(ryp) || !std::isfinite(rzp)) continue;
                double cx, cy, cz;
                if (!transformPoint(rxp, ryp, rzp, cx, cy, cz)) continue;
                if (cz < z_min_clip_m_ || cz > z_max_clip_m_) continue;
                const double ddx = cx - rx, ddy = cy - ry;
                if (ddx * ddx + ddy * ddy > max_range_sq) continue;
                pts.emplace_back(static_cast<float>(cx), static_cast<float>(cy),
                                 static_cast<float>(cz));
                minx = std::min(minx, cx);
                miny = std::min(miny, cy);
                maxx = std::max(maxx, cx);
                maxy = std::max(maxy, cy);
            }
        } catch (const std::runtime_error& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Cannot iterate cloud fields: %s", ex.what());
            return;
        }

        if (pts.empty()) return;
        if (!map_->ensure(minx, miny, maxx, maxy)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Height map at cell budget; dropping scan integration");
            return;
        }
        for (const auto& p : pts) map_->addPoint(p.x(), p.y(), p.z());
    }

    void stampDrivenFootprint() {
        if (!transform_initialized_) return;
        const double rx = latest_robot_position_.x();
        const double ry = latest_robot_position_.y();
        if (driven_stamped_once_) {
            const double dx = rx - last_stamp_x_, dy = ry - last_stamp_y_;
            if (dx * dx + dy * dy < driven_stamp_min_move_m_ * driven_stamp_min_move_m_) {
                return;
            }
        }
        const double yaw = latest_robot_yaw_;
        const double half_len = robot_length_m_ * 0.5;
        const double half_wid = robot_width_m_ * 0.5;
        const double diag = std::hypot(half_len, half_wid);
        if (!map_->ensure(rx - diag, ry - diag, rx + diag, ry + diag)) return;

        // Full footprint clears positive obstacles; wheel strips also clear holes.
        map_->stampRect(rx, ry, yaw, half_len, half_wid, /*footprint=*/true, /*wheel=*/false);
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        const double off = track_width_m_ * 0.5;
        const double wheel_half = wheel_width_m_ * 0.5;
        for (const double side : {+off, -off}) {
            // Lateral (+Y) offset in the robot frame -> world.
            const double wx = rx + (-syaw) * side;
            const double wy = ry + (cyaw) * side;
            map_->stampRect(wx, wy, yaw, half_len, wheel_half, /*footprint=*/false,
                            /*wheel=*/true);
        }
        last_stamp_x_ = rx;
        last_stamp_y_ = ry;
        driven_stamped_once_ = true;
    }

    // --- Periodic classification + publish ---

    void onPublishTick() {
        if (!armed_ || !transform_initialized_ || !odom_ready_ || map_->empty()) return;

        const double rx = latest_robot_position_.x();
        const double ry = latest_robot_position_.y();

        // Region of interest: bbox(robot, clamped-goal) padded, else a robot-
        // centred window. Clamping the goal bounds the grid the planner/EDT run on.
        double rminx = rx - window_radius_m_, rmaxx = rx + window_radius_m_;
        double rminy = ry - window_radius_m_, rmaxy = ry + window_radius_m_;
        double egx = rx, egy = ry;
        bool plan_now = have_goal_;
        if (have_goal_) {
            double dx = goal_x_ - rx, dy = goal_y_ - ry;
            const double d = std::hypot(dx, dy);
            if (d > max_plan_radius_m_ && d > 1e-6) {
                dx *= max_plan_radius_m_ / d;
                dy *= max_plan_radius_m_ / d;
            }
            egx = rx + dx;
            egy = ry + dy;
            rminx = std::min(rminx, std::min(rx, egx) - region_pad_m_);
            rmaxx = std::max(rmaxx, std::max(rx, egx) + region_pad_m_);
            rminy = std::min(rminy, std::min(ry, egy) - region_pad_m_);
            rmaxy = std::max(rmaxy, std::max(ry, egy) + region_pad_m_);
        }

        const int cgx0 = map_->clampCellX(rminx);
        const int cgx1 = map_->clampCellX(rmaxx);
        const int cgy0 = map_->clampCellY(rminy);
        const int cgy1 = map_->clampCellY(rmaxy);
        const int rw = cgx1 - cgx0 + 1;
        const int rh = cgy1 - cgy0 + 1;
        if (rw < 2 || rh < 2) return;

        const size_t rn = static_cast<size_t>(rw) * rh;
        // Per-cell gather. Two independent gates: (1) ground-confidence
        // (low-variance + enough obs) yields a trustworthy FLOOR height used as
        // the local reference and to fill FREE cells densely; (2) obstacle-
        // evidence (point-count + z extremes) flags LETHAL cells regardless of
        // variance, so tall structures (lockers/walls) that never pass the
        // ground gate are still detected.
        std::vector<float> mean(rn, 0.0f);
        std::vector<float> zmin(rn, 0.0f);
        std::vector<float> zmax(rn, 0.0f);
        std::vector<uint32_t> ncount(rn, 0);
        std::vector<uint8_t> gconf(rn, 0);  // ground (floor) confidence
        std::vector<uint8_t> drvF(rn, 0);
        std::vector<uint8_t> drvW(rn, 0);

        for (int ly = 0; ly < rh; ++ly) {
            for (int lx = 0; lx < rw; ++lx) {
                const int gi = map_->index(cgx0 + lx, cgy0 + ly);
                const int li = ly * rw + lx;
                const uint32_t n = map_->count(gi);
                drvF[li] = map_->drivenFootprint(gi) ? 1 : 0;
                drvW[li] = map_->drivenWheel(gi) ? 1 : 0;
                if (n == 0) continue;
                ncount[li] = n;
                mean[li] = map_->mean(gi);
                zmin[li] = map_->zMin(gi);
                zmax[li] = map_->zMax(gi);
                if (n >= static_cast<uint32_t>(min_obs_count_)) {
                    const double stderr_mean = std::sqrt(std::max(0.0f, map_->m2(gi))) / n;
                    if (stderr_mean <= max_stderr_m_) gconf[li] = 1;
                }
            }
        }

        // Local floor reference: lowest ground-confident mean in a small window.
        // Robust to obstacle cells (which are not ground-confident) and gives the
        // surrounding walkable height to measure rises/drops against.
        const float kNoFloor = std::numeric_limits<float>::max();
        std::vector<float> local_floor(rn, kNoFloor);
        const int R = local_floor_radius_cells_;
        for (int ly = 0; ly < rh; ++ly) {
            for (int lx = 0; lx < rw; ++lx) {
                const int li = ly * rw + lx;
                float lf = kNoFloor;
                const int y0 = std::max(0, ly - R), y1 = std::min(rh - 1, ly + R);
                const int x0 = std::max(0, lx - R), x1 = std::min(rw - 1, lx + R);
                for (int ny = y0; ny <= y1; ++ny) {
                    for (int nx = x0; nx <= x1; ++nx) {
                        const int ni = ny * rw + nx;
                        if (gconf[ni] && mean[ni] < lf) lf = mean[ni];
                    }
                }
                local_floor[li] = lf;
            }
        }

        // Costmap (3-value) + planner occupancy + heightmap display in one pass.
        std::vector<int8_t> costmap(rn, -1);   // FREE 0 / LETHAL 100 / UNKNOWN -1
        std::vector<uint8_t> occupied(rn, 0);  // planner: 1 = hard obstacle
        std::vector<float> disp_h(rn, 0.0f);   // height-map display value
        std::vector<uint8_t> disp_show(rn, 0);
        const uint32_t obs_min = static_cast<uint32_t>(obstacle_min_points_);
        for (int ly = 0; ly < rh; ++ly) {
            for (int lx = 0; lx < rw; ++lx) {
                const int li = ly * rw + lx;
                const float lf = local_floor[li];
                const bool has_floor = (lf != kNoFloor);

                // --- Eager, variance-independent evidence (tall obstacle / deep
                // drop), gated by point-count + corroborated by the cell mean so a
                // single flyer can't trip it. These are NOT driven-clearable: a
                // >8 cm wall or >20 cm hole was never driven over. ---
                bool positive = false, negative = false;
                if (has_floor && ncount[li] >= obs_min) {
                    // Base check: a real obstacle is rooted at the floor, so its
                    // lowest return sits within the overhead band. Floating /
                    // overhead returns in columns whose own floor was never
                    // observed (occluded beside the structure, or out of FOV)
                    // have a high zmin and are rejected here instead of being
                    // marked as tall obstacles.
                    if ((zmax[li] - lf) > obstacle_height_m_ &&
                        (mean[li] - lf) > obstacle_corroborate_rise_m_ &&
                        (zmin[li] - lf) <= overhead_margin_m_) {
                        positive = true;
                    }
                    if ((lf - zmin[li]) > big_drop_m_ &&
                        (lf - mean[li]) > drop_corroborate_m_) {
                        negative = true;
                    }
                }

                // --- Subtle steps from ground-confident neighbours (curbs, drains,
                // shallow depressions). Driven-clearable up to the cap. ---
                if (gconf[li] && !positive && !negative) {
                    double max_rise = 0.0, max_drop = 0.0;
                    bool has_neighbor = false;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            const int nx = lx + dx, ny = ly + dy;
                            if (nx < 0 || ny < 0 || nx >= rw || ny >= rh) continue;
                            const int ni = ny * rw + nx;
                            if (!gconf[ni]) continue;
                            has_neighbor = true;
                            const double diff = mean[li] - mean[ni];
                            max_rise = std::max(max_rise, diff);
                            max_drop = std::max(max_drop, -diff);
                        }
                    }
                    bool pstep = has_neighbor && (max_rise > step_up_m_);
                    bool nstep = has_neighbor && (max_drop > step_down_m_);
                    if (pstep && (drvF[li] || drvW[li]) &&
                        max_rise <= driven_override_max_step_m_) {
                        pstep = false;
                    }
                    if (nstep && drvW[li] && max_drop <= driven_override_max_step_m_) {
                        nstep = false;
                    }
                    positive = positive || pstep;
                    negative = negative || nstep;
                }

                const bool lethal = positive || negative;
                if (lethal) {
                    costmap[li] = 100;
                    occupied[li] = 1;
                    disp_h[li] = negative ? zmin[li] : zmax[li];
                    disp_show[li] = 1;
                } else if (gconf[li]) {
                    costmap[li] = 0;  // FREE: trustworthy floor
                    occupied[li] = 0;
                    disp_h[li] = mean[li];
                    disp_show[li] = 1;
                } else if (drvF[li]) {
                    costmap[li] = 0;  // driven footprint => known-traversable
                    occupied[li] = 0;
                    if (ncount[li] > 0) {
                        disp_h[li] = mean[li];
                        disp_show[li] = 1;
                    }
                } else {
                    costmap[li] = -1;  // UNKNOWN
                    occupied[li] = plan_through_unknown_ ? 0 : 1;
                }
            }
        }

        const double origin_x = map_->cellCenterX(cgx0) - 0.5 * cell_size_m_;
        const double origin_y = map_->cellCenterY(cgy0) - 0.5 * cell_size_m_;
        publishCostmap(costmap, rw, rh, origin_x, origin_y);

        if (plan_now) {
            planAndPublish(occupied, rw, rh, origin_x, origin_y, rx, ry, egx, egy);
        }

        if (publish_heightmap_ && heightmap_pub_ && heightmapDue()) {
            publishHeightmap(disp_h, disp_show, rw, rh, cgx0, cgy0);
        }
    }

    void publishCostmap(const std::vector<int8_t>& data, int w, int h, double ox,
                        double oy) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = frameId();
        grid.info.resolution = static_cast<float>(cell_size_m_);
        grid.info.width = static_cast<uint32_t>(w);
        grid.info.height = static_cast<uint32_t>(h);
        grid.info.origin.position.x = ox;
        grid.info.origin.position.y = oy;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(data.begin(), data.end());
        costmap_pub_->publish(grid);
    }

    void planAndPublish(const std::vector<uint8_t>& occupied, int w, int h, double ox,
                        double oy, double rx, double ry, double gx, double gy) {
        pilot_control::RoofEdgeGridPlanner planner;
        if (!planner.buildFromGrid(occupied, w, h, ox, oy, cell_size_m_,
                                   inflation_radius_m_)) {
            return;
        }
        const std::vector<pilot_control::Point2D> path =
            planner.plan({rx, ry}, {gx, gy}, snap_radius_m_, /*bias_clearance=*/false);

        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frameId();
        msg.poses.reserve(path.size());
        for (const auto& p : path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        if (path.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No path to goal [%.2f, %.2f] (blocked / unreachable)", gx,
                                 gy);
        }
        path_pub_->publish(msg);
    }

    bool heightmapDue() {
        const auto now = std::chrono::steady_clock::now();
        const double period = (heightmap_rate_hz_ > 0.0) ? 1.0 / heightmap_rate_hz_ : 1.0;
        if (heightmap_published_once_ &&
            std::chrono::duration<double>(now - last_heightmap_tp_).count() < period) {
            return false;
        }
        last_heightmap_tp_ = now;
        heightmap_published_once_ = true;
        return true;
    }

    void publishHeightmap(const std::vector<float>& h, const std::vector<uint8_t>& conf,
                          int w, int hgt, int cgx0, int cgy0) {
        std::vector<std::array<float, 4>> points;  // x, y, z(=ground), intensity(=ground)
        points.reserve(static_cast<size_t>(w) * hgt / (heightmap_decimate_ * heightmap_decimate_));
        for (int ly = 0; ly < hgt; ly += heightmap_decimate_) {
            for (int lx = 0; lx < w; lx += heightmap_decimate_) {
                const int li = ly * w + lx;
                if (!conf[li]) continue;
                const float px = static_cast<float>(map_->cellCenterX(cgx0 + lx));
                const float py = static_cast<float>(map_->cellCenterY(cgy0 + ly));
                points.push_back({px, py, h[li], h[li]});
            }
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = frameId();
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                 sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                 sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> oy(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> oz(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> oi(cloud, "intensity");
        for (const auto& p : points) {
            *ox = p[0];
            *oy = p[1];
            *oz = p[2];
            *oi = p[3];
            ++ox;
            ++oy;
            ++oz;
            ++oi;
        }
        heightmap_pub_->publish(cloud);
    }

    std::string frameId() const {
        return corrected_frame_id_.empty() ? std::string("camera_init") : corrected_frame_id_;
    }

    // Parameters.
    std::string cloud_topic_, raw_odometry_topic_, corrected_odometry_topic_, goal_topic_;
    std::string costmap_topic_, heightmap_topic_, path_topic_, calibration_file_;
    std::string lidar_world_frame_;
    double cell_size_m_, step_up_m_, step_down_m_, driven_override_max_step_m_, max_stderr_m_;
    int min_obs_count_;
    int obstacle_min_points_, local_floor_radius_cells_;
    double obstacle_height_m_, obstacle_corroborate_rise_m_, big_drop_m_, drop_corroborate_m_;
    double overhead_margin_m_;
    double robot_length_m_, robot_width_m_, track_width_m_, wheel_width_m_, inflation_radius_m_;
    double window_radius_m_, max_plan_radius_m_, region_pad_m_, max_range_m_;
    double z_min_clip_m_, z_max_clip_m_;
    double publish_rate_hz_, heightmap_rate_hz_, grow_pad_m_, driven_stamp_min_move_m_;
    double snap_radius_m_;
    int heightmap_decimate_, cloud_decimate_;
    bool publish_heightmap_, plan_through_unknown_;
    long long max_map_cells_;

    // State.
    bool armed_ = false;
    bool odom_ready_ = false;
    bool transform_initialized_ = false;
    Eigen::Matrix3d r_init_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p0_lidar_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_robot_position_ = Eigen::Vector3d::Zero();
    double latest_robot_yaw_ = 0.0;
    std::string corrected_frame_id_;

    uint64_t pending_corrected_stamp_ns_ = 0;
    Eigen::Vector3d pending_corrected_position_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d pending_corrected_orientation_ = Eigen::Matrix3d::Identity();
    bool pending_corrected_valid_ = false;
    std::deque<RawOdomSample> raw_odom_buffer_;

    bool have_goal_ = false;
    double goal_x_ = 0.0, goal_y_ = 0.0;

    bool driven_stamped_once_ = false;
    double last_stamp_x_ = 0.0, last_stamp_y_ = 0.0;

    bool heightmap_published_once_ = false;
    std::chrono::steady_clock::time_point last_heightmap_tp_{};

    std::unique_ptr<PersistentHeightMap> map_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoofEdgeCostmapNode>());
    rclcpp::shutdown();
    return 0;
}
