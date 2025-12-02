/**
 * @file coverage_pipeline.cpp
 * @brief Implementation of coverage planning pipeline
 */

#include "coverage_pipeline.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#ifdef HAVE_FIELDS2COVER
#include <fields2cover.h>
#endif

#ifdef HAVE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/convex_hull_2.h>
#endif

namespace f2c_cpp {

// Global progress callback
static ProgressCallback g_progressCallback = nullptr;

void setProgressCallback(ProgressCallback callback) {
    g_progressCallback = callback;
}

static void reportProgress(int percent, const std::string& message) {
    if (g_progressCallback) {
        g_progressCallback(percent, message);
    }
}

// =============================================================================
// Point Cloud Processing
// =============================================================================

PointCloudPtr loadPointCloudFile(const std::string& path) {
    PointCloudPtr cloud(new PointCloud);
    
    // Determine file type by extension
    std::string ext = path.substr(path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    int result = -1;
    if (ext == "pcd") {
        result = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    } else if (ext == "ply") {
        result = pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud);
    } else if (ext == "xyz") {
        // Simple XYZ text format
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + path);
        }
        double x, y, z;
        while (file >> x >> y >> z) {
            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
        result = cloud->empty() ? -1 : 0;
    } else {
        throw std::runtime_error("Unsupported file format: " + ext);
    }
    
    if (result < 0 || cloud->empty()) {
        throw std::runtime_error("Failed to load point cloud: " + path);
    }
    
    return cloud;
}

PointCloudPtr filterByZBand(const PointCloudPtr& cloud, double z_band) {
    if (z_band <= 0 || cloud->empty()) {
        return cloud;
    }
    
    // Compute median Z
    std::vector<float> z_values;
    z_values.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        z_values.push_back(pt.z);
    }
    std::sort(z_values.begin(), z_values.end());
    float z_median = z_values[z_values.size() / 2];
    
    // Filter points within band
    PointCloudPtr filtered(new PointCloud);
    filtered->reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        if (std::abs(pt.z - z_median) <= z_band) {
            filtered->push_back(pt);
        }
    }
    
    if (filtered->empty()) {
        throw std::runtime_error("Z-band filter removed all points");
    }
    
    return filtered;
}

PointCloudPtr filterByZRange(const PointCloudPtr& cloud, double z_min, double z_max) {
    if (cloud->empty()) {
        return cloud;
    }
    
    // Swap if min > max
    if (z_min > z_max) {
        std::swap(z_min, z_max);
    }
    
    // Filter points within [z_min, z_max] range (relative to origin Z=0)
    PointCloudPtr filtered(new PointCloud);
    filtered->reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        if (pt.z >= z_min && pt.z <= z_max) {
            filtered->push_back(pt);
        }
    }
    
    if (filtered->empty()) {
        throw std::runtime_error("Z-range filter removed all points. Try adjusting Z min/max values.");
    }
    
    return filtered;
}

PointCloudPtr subsampleRandom(const PointCloudPtr& cloud, size_t max_points) {
    if (max_points == 0 || cloud->size() <= max_points) {
        return cloud;
    }
    
    std::vector<size_t> indices(cloud->size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);
    
    PointCloudPtr sampled(new PointCloud);
    sampled->reserve(max_points);
    for (size_t i = 0; i < max_points; ++i) {
        sampled->push_back(cloud->points[indices[i]]);
    }
    
    return sampled;
}

PointCloudPtr downsampleVoxel(const PointCloudPtr& cloud, double voxel_size) {
    if (voxel_size <= 0) {
        return cloud;
    }
    
    PointCloudPtr filtered(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*filtered);
    
    return filtered;
}

PointCloudPtr downsampleStatistical(const PointCloudPtr& cloud, int mean_k, double std_ratio) {
    if (mean_k <= 0 || std_ratio <= 0) {
        return cloud;
    }
    
    PointCloudPtr filtered(new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_ratio);
    sor.filter(*filtered);
    
    return filtered;
}

// =============================================================================
// Geometry Operations
// =============================================================================

Polygon2D computeConvexHull(const std::vector<Point2D>& points) {
    if (points.size() < 3) {
        throw std::runtime_error("Need at least 3 points for convex hull");
    }
    
    // Simple Graham scan implementation
    std::vector<Point2D> sorted_points = points;
    
    // Find lowest point
    auto lowest = std::min_element(sorted_points.begin(), sorted_points.end(),
        [](const Point2D& a, const Point2D& b) {
            return (a.y < b.y) || (a.y == b.y && a.x < b.x);
        });
    std::swap(*sorted_points.begin(), *lowest);
    Point2D pivot = sorted_points[0];
    
    // Sort by polar angle
    std::sort(sorted_points.begin() + 1, sorted_points.end(),
        [&pivot](const Point2D& a, const Point2D& b) {
            double angle_a = std::atan2(a.y - pivot.y, a.x - pivot.x);
            double angle_b = std::atan2(b.y - pivot.y, b.x - pivot.x);
            return angle_a < angle_b;
        });
    
    // Build hull
    Polygon2D hull;
    for (const auto& p : sorted_points) {
        while (hull.size() >= 2) {
            Point2D& p1 = hull[hull.size() - 2];
            Point2D& p2 = hull[hull.size() - 1];
            double cross = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
            if (cross <= 0) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(p);
    }
    
    return hull;
}

// Forward declarations for CGAL functions
#ifdef HAVE_CGAL
static Polygon2D computeAlphaShapeImpl(const std::vector<Point2D>& points, double alpha);
static Polygon2D computeDelaunayBoundaryImpl(const std::vector<Point2D>& points);
static Polygon2D computeGridBoundaryImpl(const std::vector<Point2D>& points, double grid_size);
#endif

Polygon2D computeConcaveHull(const std::vector<Point2D>& points, 
                             double alpha,
                             const std::string& method) {
    if (points.size() < 3) {
        throw std::runtime_error("Need at least 3 points for hull computation");
    }
    
    reportProgress(10, "Computing hull using " + method + "...");
    
    std::string m = method;
    std::transform(m.begin(), m.end(), m.begin(), ::tolower);
    
#ifdef HAVE_CGAL
    if (m == "alphashape" || m == "alpha") {
        return computeAlphaShapeImpl(points, alpha);
    } else if (m == "delaunay") {
        return computeDelaunayBoundaryImpl(points);
    } else if (m == "grid") {
        return computeGridBoundaryImpl(points, alpha);
    }
#endif
    
    // Fallback to convex hull
    reportProgress(50, "Using convex hull fallback");
    Polygon2D result = computeConvexHull(points);
    reportProgress(100, "Hull computed");
    
    return result;
}

#ifdef HAVE_CGAL
// CGAL type aliases (inside namespace)
using CGALKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGALPoint = CGALKernel::Point_2;
using CGALVb = CGAL::Alpha_shape_vertex_base_2<CGALKernel>;
using CGALFb = CGAL::Alpha_shape_face_base_2<CGALKernel>;
using CGALTds = CGAL::Triangulation_data_structure_2<CGALVb, CGALFb>;
using CGALTriangulation = CGAL::Delaunay_triangulation_2<CGALKernel, CGALTds>;
using CGALAlphaShape = CGAL::Alpha_shape_2<CGALTriangulation>;

static Polygon2D computeAlphaShapeImpl(const std::vector<Point2D>& points, double alpha) {
    
    reportProgress(20, "Building alpha shape...");
    
    // Convert points to CGAL format
    std::vector<CGALPoint> cgal_points;
    cgal_points.reserve(points.size());
    for (const auto& p : points) {
        cgal_points.emplace_back(p.x, p.y);
    }
    
    // Build alpha shape - alpha parameter: smaller = more detail
    // CGAL uses alpha^2 internally, so we need to adjust
    double alpha_value = 1.0 / (alpha * alpha);  // Smaller alpha param = larger alpha_value = more detail
    
    CGALAlphaShape A(cgal_points.begin(), cgal_points.end(), alpha_value, CGALAlphaShape::GENERAL);
    
    reportProgress(50, "Extracting boundary...");
    
    // Extract boundary edges
    std::vector<std::pair<CGALPoint, CGALPoint>> edges;
    for (auto it = A.alpha_shape_edges_begin(); it != A.alpha_shape_edges_end(); ++it) {
        auto face = it->first;
        int i = it->second;
        CGALPoint p1 = face->vertex((i + 1) % 3)->point();
        CGALPoint p2 = face->vertex((i + 2) % 3)->point();
        edges.emplace_back(p1, p2);
    }
    
    if (edges.empty()) {
        reportProgress(100, "Alpha shape empty, using convex hull");
        return computeConvexHull(points);
    }
    
    reportProgress(70, "Ordering boundary points...");
    
    // Build adjacency map and order boundary
    std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> adj;
    for (const auto& e : edges) {
        double x1 = CGAL::to_double(e.first.x());
        double y1 = CGAL::to_double(e.first.y());
        double x2 = CGAL::to_double(e.second.x());
        double y2 = CGAL::to_double(e.second.y());
        auto p1 = std::make_pair(x1, y1);
        auto p2 = std::make_pair(x2, y2);
        adj[p1].push_back(p2);
        adj[p2].push_back(p1);
    }
    
    // Find the longest connected boundary
    Polygon2D best_boundary;
    std::set<std::pair<double, double>> global_visited;
    
    for (const auto& start_pair : adj) {
        if (global_visited.count(start_pair.first)) continue;
        
        Polygon2D current_boundary;
        auto current = start_pair.first;
        std::set<std::pair<double, double>> local_visited;
        
        while (local_visited.find(current) == local_visited.end()) {
            local_visited.insert(current);
            global_visited.insert(current);
            current_boundary.emplace_back(current.first, current.second);
            
            bool found = false;
            for (const auto& next : adj[current]) {
                if (local_visited.find(next) == local_visited.end()) {
                    current = next;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }
        
        if (current_boundary.size() > best_boundary.size()) {
            best_boundary = current_boundary;
        }
    }
    
    if (best_boundary.size() < 3) {
        reportProgress(100, "Alpha boundary too small, using convex hull");
        return computeConvexHull(points);
    }
    
    reportProgress(100, "Alpha shape computed");
    return best_boundary;
}

static Polygon2D computeDelaunayBoundaryImpl(const std::vector<Point2D>& points) {
    
    reportProgress(20, "Building Delaunay triangulation...");
    
    // Convert points to CGAL format
    std::vector<CGALPoint> cgal_points;
    cgal_points.reserve(points.size());
    for (const auto& p : points) {
        cgal_points.emplace_back(p.x, p.y);
    }
    
    // Build Delaunay triangulation
    CGALTriangulation dt(cgal_points.begin(), cgal_points.end());
    
    reportProgress(50, "Finding boundary edges...");
    
    // Count edge occurrences - boundary edges appear once
    std::map<std::pair<std::pair<double,double>, std::pair<double,double>>, int> edge_count;
    
    for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
        for (int i = 0; i < 3; ++i) {
            auto p1 = fit->vertex((i + 1) % 3)->point();
            auto p2 = fit->vertex((i + 2) % 3)->point();
            
            double x1 = CGAL::to_double(p1.x());
            double y1 = CGAL::to_double(p1.y());
            double x2 = CGAL::to_double(p2.x());
            double y2 = CGAL::to_double(p2.y());
            
            auto key1 = std::make_pair(x1, y1);
            auto key2 = std::make_pair(x2, y2);
            
            if (key1 > key2) std::swap(key1, key2);
            auto edge_key = std::make_pair(key1, key2);
            edge_count[edge_key]++;
        }
    }
    
    // Extract boundary edges (those that appear only once)
    std::map<std::pair<double, double>, std::vector<std::pair<double, double>>> adj;
    for (const auto& ec : edge_count) {
        if (ec.second == 1) {
            adj[ec.first.first].push_back(ec.first.second);
            adj[ec.first.second].push_back(ec.first.first);
        }
    }
    
    if (adj.empty()) {
        return computeConvexHull(points);
    }
    
    reportProgress(70, "Ordering boundary...");
    
    // Order boundary vertices
    Polygon2D boundary;
    auto start = adj.begin()->first;
    auto current = start;
    std::set<std::pair<double, double>> visited;
    
    while (visited.find(current) == visited.end()) {
        visited.insert(current);
        boundary.emplace_back(current.first, current.second);
        
        bool found = false;
        for (const auto& next : adj[current]) {
            if (visited.find(next) == visited.end()) {
                current = next;
                found = true;
                break;
            }
        }
        if (!found) break;
    }
    
    reportProgress(100, "Delaunay boundary computed");
    
    if (boundary.size() < 3) {
        return computeConvexHull(points);
    }
    
    return boundary;
}

static Polygon2D computeGridBoundaryImpl(const std::vector<Point2D>& points, double grid_size) {
    reportProgress(20, "Computing grid boundary...");
    
    if (points.size() < 3) {
        return computeConvexHull(points);
    }
    
    // Find bounds
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    
    // Auto grid size if needed
    if (grid_size <= 0) {
        double range = std::max(max_x - min_x, max_y - min_y);
        grid_size = range / 100.0;
    }
    
    // Add padding
    double padding = grid_size * 2;
    min_x -= padding; min_y -= padding;
    max_x += padding; max_y += padding;
    
    int grid_w = static_cast<int>((max_x - min_x) / grid_size) + 1;
    int grid_h = static_cast<int>((max_y - min_y) / grid_size) + 1;
    
    reportProgress(40, "Building occupancy grid...");
    
    // Build occupancy grid
    std::vector<std::vector<bool>> grid(grid_h, std::vector<bool>(grid_w, false));
    for (const auto& p : points) {
        int xi = static_cast<int>((p.x - min_x) / grid_size);
        int yi = static_cast<int>((p.y - min_y) / grid_size);
        xi = std::clamp(xi, 0, grid_w - 1);
        yi = std::clamp(yi, 0, grid_h - 1);
        grid[yi][xi] = true;
        // Also fill neighbors for robustness
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = xi + dx, ny = yi + dy;
                if (nx >= 0 && nx < grid_w && ny >= 0 && ny < grid_h) {
                    grid[ny][nx] = true;
                }
            }
        }
    }
    
    reportProgress(60, "Finding boundary cells...");
    
    // Find boundary cells (occupied cells with at least one empty neighbor)
    std::vector<Point2D> boundary_points;
    for (int y = 0; y < grid_h; ++y) {
        for (int x = 0; x < grid_w; ++x) {
            if (grid[y][x]) {
                bool is_boundary = false;
                for (int dy = -1; dy <= 1 && !is_boundary; ++dy) {
                    for (int dx = -1; dx <= 1 && !is_boundary; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || nx >= grid_w || ny < 0 || ny >= grid_h || !grid[ny][nx]) {
                            is_boundary = true;
                        }
                    }
                }
                if (is_boundary) {
                    boundary_points.emplace_back(min_x + x * grid_size, min_y + y * grid_size);
                }
            }
        }
    }
    
    if (boundary_points.size() < 3) {
        return computeConvexHull(points);
    }
    
    reportProgress(80, "Computing boundary hull...");
    
    // Use convex hull of boundary points
    Polygon2D result = computeConvexHull(boundary_points);
    
    reportProgress(100, "Grid boundary computed");
    return result;
}

#endif // HAVE_CGAL

Polygon2D simplifyPolygon(const Polygon2D& poly, double tolerance) {
    if (tolerance <= 0 || poly.size() < 3) {
        return poly;
    }
    
    // Douglas-Peucker algorithm
    std::function<void(int, int, std::vector<bool>&)> douglasPeucker;
    douglasPeucker = [&](int start, int end, std::vector<bool>& keep) {
        if (end - start < 2) return;
        
        double max_dist = 0;
        int max_idx = start;
        
        const Point2D& p1 = poly[start];
        const Point2D& p2 = poly[end];
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double len_sq = dx * dx + dy * dy;
        
        for (int i = start + 1; i < end; ++i) {
            double dist;
            if (len_sq < 1e-10) {
                dist = std::hypot(poly[i].x - p1.x, poly[i].y - p1.y);
            } else {
                double t = ((poly[i].x - p1.x) * dx + (poly[i].y - p1.y) * dy) / len_sq;
                t = std::max(0.0, std::min(1.0, t));
                double proj_x = p1.x + t * dx;
                double proj_y = p1.y + t * dy;
                dist = std::hypot(poly[i].x - proj_x, poly[i].y - proj_y);
            }
            
            if (dist > max_dist) {
                max_dist = dist;
                max_idx = i;
            }
        }
        
        if (max_dist > tolerance) {
            keep[max_idx] = true;
            douglasPeucker(start, max_idx, keep);
            douglasPeucker(max_idx, end, keep);
        }
    };
    
    std::vector<bool> keep(poly.size(), false);
    keep[0] = true;
    keep[poly.size() - 1] = true;
    douglasPeucker(0, poly.size() - 1, keep);
    
    Polygon2D simplified;
    for (size_t i = 0; i < poly.size(); ++i) {
        if (keep[i]) {
            simplified.push_back(poly[i]);
        }
    }
    
    return simplified;
}

double computeSwathAngle(const Polygon2D& poly) {
    if (poly.size() < 3) return 0.0;
    
    double best_angle = 0.0;
    double min_area = std::numeric_limits<double>::max();
    
    // Try angles from each edge
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        
        double angle = std::atan2(p2.y - p1.y, p2.x - p1.x);
        
        // Rotate points and find bounding box
        double cos_a = std::cos(-angle);
        double sin_a = std::sin(-angle);
        
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& p : poly) {
            double rx = p.x * cos_a - p.y * sin_a;
            double ry = p.x * sin_a + p.y * cos_a;
            min_x = std::min(min_x, rx);
            max_x = std::max(max_x, rx);
            min_y = std::min(min_y, ry);
            max_y = std::max(max_y, ry);
        }
        
        double area = (max_x - min_x) * (max_y - min_y);
        double width = max_x - min_x;
        double height = max_y - min_y;
        
        if (area < min_area) {
            min_area = area;
            best_angle = (width > height) ? angle : angle + M_PI / 2.0;
        }
    }
    
    // Normalize to [0, pi)
    while (best_angle < 0) best_angle += M_PI;
    while (best_angle >= M_PI) best_angle -= M_PI;
    
    return best_angle;
}

bool isPolygonConcave(const Polygon2D& poly) {
    if (poly.size() < 3) return false;
    
    bool has_positive = false;
    bool has_negative = false;
    
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        const Point2D& p3 = poly[(i + 2) % poly.size()];
        
        double cross = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        
        if (cross > 1e-10) has_positive = true;
        if (cross < -1e-10) has_negative = true;
    }
    
    return has_positive && has_negative;
}

double polygonArea(const Polygon2D& poly) {
    if (poly.size() < 3) return 0.0;
    
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        area += (p1.x * p2.y - p2.x * p1.y);
    }
    
    return std::abs(area) / 2.0;
}

bool isPolygonValid(const Polygon2D& poly) {
    if (poly.size() < 3) return false;
    
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        if (std::hypot(p2.x - p1.x, p2.y - p1.y) < 1e-10) {
            return false;
        }
    }
    
    return polygonArea(poly) > 1e-10;
}

void polygonBounds(const Polygon2D& poly, 
                   double& min_x, double& min_y, 
                   double& max_x, double& max_y) {
    if (poly.empty()) {
        min_x = min_y = max_x = max_y = 0;
        return;
    }
    
    min_x = max_x = poly[0].x;
    min_y = max_y = poly[0].y;
    
    for (const auto& p : poly) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
}

// =============================================================================
// Coverage Generation
// =============================================================================

PathStateList swathsToAxialTurnPath(const SwathList& swaths) {
    PathStateList path;
    
    for (const auto& swath : swaths) {
        double heading = swath.heading;
        double vx = std::cos(heading);
        double vy = std::sin(heading);
        
        PathState start_state;
        start_state.point = swath.start;
        start_state.heading = heading;
        start_state.vx = vx;
        start_state.vy = vy;
        path.push_back(start_state);
        
        PathState end_state;
        end_state.point = swath.end;
        end_state.heading = heading;
        end_state.vx = vx;
        end_state.vy = vy;
        path.push_back(end_state);
    }
    
    return path;
}

// Helper: Calculate signed area of polygon (positive = CCW, negative = CW)
static double signedPolygonArea(const Polygon2D& poly) {
    if (poly.size() < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        area += (p1.x * p2.y - p2.x * p1.y);
    }
    return area / 2.0;
}

// Helper: Check if polygon is clockwise (for interior ring)
static bool isClockwise(const Polygon2D& poly) {
    return signedPolygonArea(poly) < 0;
}

// Helper: Check if point is inside polygon (ray casting)
static bool pointInPolygon(const Point2D& p, const Polygon2D& poly) {
    if (poly.size() < 3) return false;
    
    int crossings = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        
        if ((p1.y <= p.y && p2.y > p.y) || (p2.y <= p.y && p1.y > p.y)) {
            double x_intersect = p1.x + (p.y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
            if (p.x < x_intersect) {
                crossings++;
            }
        }
    }
    return (crossings % 2) == 1;
}

// Helper: Check if line segment intersects polygon edge
static bool segmentIntersectsPolygon(const Point2D& a, const Point2D& b, const Polygon2D& poly) {
    // Check if either endpoint is inside the polygon
    if (pointInPolygon(a, poly) || pointInPolygon(b, poly)) {
        return true;
    }
    
    // Check if line segment crosses any edge
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point2D& p1 = poly[i];
        const Point2D& p2 = poly[(i + 1) % poly.size()];
        
        // Line segment intersection test
        double d1x = b.x - a.x, d1y = b.y - a.y;
        double d2x = p2.x - p1.x, d2y = p2.y - p1.y;
        double cross = d1x * d2y - d1y * d2x;
        
        if (std::abs(cross) < 1e-10) continue;  // Parallel
        
        double t = ((p1.x - a.x) * d2y - (p1.y - a.y) * d2x) / cross;
        double u = ((p1.x - a.x) * d1y - (p1.y - a.y) * d1x) / cross;
        
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            return true;
        }
    }
    return false;
}

// Filter path to avoid obstacles - removes points inside obstacles
// and adds intermediate points to go around them
static PathStateList filterPathAroundObstacles(
    const PathStateList& original_path,
    const std::vector<Polygon2D>& obstacles) {
    
    if (obstacles.empty() || original_path.empty()) {
        return original_path;
    }
    
    PathStateList filtered;
    filtered.reserve(original_path.size());
    
    for (size_t i = 0; i < original_path.size(); ++i) {
        const PathState& state = original_path[i];
        
        // Check if this point is inside any obstacle
        bool inside_obstacle = false;
        for (const auto& obs : obstacles) {
            if (pointInPolygon(state.point, obs)) {
                inside_obstacle = true;
                break;
            }
        }
        
        if (!inside_obstacle) {
            // Check if segment from previous point crosses obstacle
            if (!filtered.empty()) {
                const Point2D& prev = filtered.back().point;
                bool crosses_obstacle = false;
                
                for (const auto& obs : obstacles) {
                    if (segmentIntersectsPolygon(prev, state.point, obs)) {
                        crosses_obstacle = true;
                        break;
                    }
                }
                
                if (crosses_obstacle) {
                    // Skip this point - it creates a path through obstacle
                    // The axial turn path will be used as fallback
                    continue;
                }
            }
            filtered.push_back(state);
        }
    }
    
    return filtered;
}

#ifdef HAVE_FIELDS2COVER

CoverageResult generateCoverage(const Polygon2D& boundary,
                                const CoverageConfig& config,
                                const Polygon2D* roi,
                                const std::vector<Polygon2D>* obstacles) {
    CoverageResult result;
    
    try {
        reportProgress(5, "Building F2C field...");
        
        // Use ROI if provided
        const Polygon2D& effective_poly = (roi && !roi->empty()) ? *roi : boundary;
        
        // Convert polygon to F2C types
        F2CLinearRing ring;
        for (const auto& p : effective_poly) {
            ring.addPoint(p.x, p.y);
        }
        
        F2CCell cell;
        cell.addRing(ring);  // Exterior ring
        
        // Add obstacles as interior rings (holes)
        // Interior rings must have clockwise winding order (negative area)
        // Check each obstacle's winding and reverse if necessary
        if (obstacles && !obstacles->empty()) {
            reportProgress(8, "Adding obstacles...");
            for (const auto& obs : *obstacles) {
                if (obs.size() >= 3) {
                    F2CLinearRing obs_ring;
                    
                    // Check if obstacle is already clockwise (correct for interior ring)
                    if (isClockwise(obs)) {
                        // Already clockwise - add as-is
                        for (const auto& p : obs) {
                            obs_ring.addPoint(p.x, p.y);
                        }
                    } else {
                        // Counter-clockwise - reverse to make clockwise
                        for (auto it = obs.rbegin(); it != obs.rend(); ++it) {
                            obs_ring.addPoint(it->x, it->y);
                        }
                    }
                    cell.addRing(obs_ring);  // Interior ring (hole)
                }
            }
        }
        
        F2CCells cells;
        cells.addGeometry(cell);
        
        F2CField field(cells);
        
        // Generate headlands
        reportProgress(15, "Generating headlands...");
        F2CCells working_area = cells;
        if (config.headland_width > 0) {
            try {
                f2c::hg::ConstHL hl_gen;
                auto cropped = hl_gen.generateHeadlands(cells, config.headland_width);
                if (cropped.size() > 0) {
                    working_area = cropped;
                }
            } catch (...) {
                // Keep original cells if headland generation fails
            }
        }
        
        // Generate swaths
        reportProgress(30, "Generating swaths...");
        f2c::sg::BruteForce sg;
        
        // Compute swath direction
        double direction = M_PI;
        if (config.auto_align) {
            double base_angle = computeSwathAngle(effective_poly);
            direction = (config.align_mode == "long") ? base_angle : base_angle + M_PI / 2.0;
        }
        
        F2CSwaths f2c_swaths;
        try {
            f2c_swaths = sg.generateSwaths(direction, config.swath_width, working_area.getGeometry(0));
        } catch (...) {
            for (size_t i = 0; i < working_area.size(); ++i) {
                auto part = sg.generateSwaths(direction, config.swath_width, working_area.getGeometry(i));
                for (size_t j = 0; j < part.size(); ++j) {
                    f2c_swaths.push_back(part.at(j));
                }
            }
        }
        
        if (f2c_swaths.size() == 0) {
            result.error_message = "No swaths generated";
            return result;
        }
        
        // Convert swaths to our format
        for (size_t i = 0; i < f2c_swaths.size(); ++i) {
            auto& sw = f2c_swaths.at(i);
            Swath swath;
            swath.start = Point2D(sw.startPoint().getX(), sw.startPoint().getY());
            swath.end = Point2D(sw.endPoint().getX(), sw.endPoint().getY());
            swath.heading = std::atan2(swath.end.y - swath.start.y, swath.end.x - swath.start.x);
            result.swaths.push_back(swath);
        }
        
        // Generate route
        reportProgress(50, "Generating route...");
        try {
            F2CSwathsByCells swaths_by_cells;
            swaths_by_cells.push_back(f2c_swaths);
            
            // Sort swaths
            F2CSwaths sorted_swaths = f2c_swaths;
            f2c::rp::BoustrophedonOrder sorter;
            sorted_swaths = sorter.genSortedSwaths(f2c_swaths);
            
            swaths_by_cells = F2CSwathsByCells();
            swaths_by_cells.push_back(sorted_swaths);
            
            f2c::rp::RoutePlannerBase route_planner;
            F2CRoute f2c_route = route_planner.genRoute(working_area, swaths_by_cells);
            
            // Update swaths with sorted order
            result.swaths.clear();
            for (size_t i = 0; i < sorted_swaths.size(); ++i) {
                auto& sw = sorted_swaths.at(i);
                Swath swath;
                swath.start = Point2D(sw.startPoint().getX(), sw.startPoint().getY());
                swath.end = Point2D(sw.endPoint().getX(), sw.endPoint().getY());
                swath.heading = std::atan2(swath.end.y - swath.start.y, swath.end.x - swath.start.x);
                result.swaths.push_back(swath);
            }
            
            // Extract route waypoints
            auto route_line = f2c_route.asLineString();
            for (size_t i = 0; i < route_line.size(); ++i) {
                PathState state;
                state.point = Point2D(route_line.getX(i), route_line.getY(i));
                result.route.push_back(state);
            }
            
            // Generate path
            reportProgress(70, "Generating path...");
            
            // When obstacles are present, use axial turns because:
            // 1. Dubins/smooth curves may cross obstacles during turns
            // 2. Swaths already avoid obstacles (via interior rings)
            // 3. Axial turns go directly between swath endpoints
            bool use_axial = config.use_axial_turns || (obstacles && !obstacles->empty());
            
            if (use_axial) {
                // For axial turns, use the route waypoints directly
                // The route already contains the correct traversal order with proper
                // direction for each swath (Boustrophedon alternates directions)
                if (!result.route.empty()) {
                    // Route waypoints are already in correct order - use them as path
                    result.path = result.route;
                    
                    // Calculate headings between consecutive waypoints
                    for (size_t i = 0; i < result.path.size(); ++i) {
                        double heading;
                        if (i + 1 < result.path.size()) {
                            // Use direction to next point
                            double dx = result.path[i+1].point.x - result.path[i].point.x;
                            double dy = result.path[i+1].point.y - result.path[i].point.y;
                            heading = std::atan2(dy, dx);
                        } else if (i > 0) {
                            // Last point - use same heading as arrival
                            heading = result.path[i-1].heading;
                        } else {
                            heading = 0;
                        }
                        result.path[i].heading = heading;
                        result.path[i].vx = std::cos(heading);
                        result.path[i].vy = std::sin(heading);
                    }
                } else {
                    // Fallback: use stored swaths
                    result.path = swathsToAxialTurnPath(result.swaths);
                }
            } else {
                // Use path planner based on config
                // "none" means straight lines (use route directly like axial turns)
                if (config.path_planner == "none") {
                    // Straight path - use route waypoints directly
                    if (!result.route.empty()) {
                        result.path = result.route;
                        // Calculate headings between consecutive waypoints
                        for (size_t i = 0; i < result.path.size(); ++i) {
                            double heading;
                            if (i + 1 < result.path.size()) {
                                double dx = result.path[i+1].point.x - result.path[i].point.x;
                                double dy = result.path[i+1].point.y - result.path[i].point.y;
                                heading = std::atan2(dy, dx);
                            } else if (i > 0) {
                                heading = result.path[i-1].heading;
                            } else {
                                heading = 0;
                            }
                            result.path[i].heading = heading;
                            result.path[i].vx = std::cos(heading);
                            result.path[i].vy = std::sin(heading);
                        }
                    } else {
                        result.path = swathsToAxialTurnPath(result.swaths);
                    }
                } else {
                    // Use smooth curves (Dubins or Reeds-Shepp)
                    try {
                        F2CRobot robot(config.swath_width, config.swath_width);
                        robot.setMinTurningRadius(config.turn_radius);
                        
                        f2c::pp::PathPlanning pp;
                        F2CPath f2c_path;
                        
                        // Select path planner based on config
                        if (config.path_planner == "dubins") {
                            f2c::pp::DubinsCurves planner;
                            f2c_path = pp.planPath(robot, f2c_route, planner);
                        } else if (config.path_planner == "dubins_cc") {
                            f2c::pp::DubinsCurvesCC planner;
                            f2c_path = pp.planPath(robot, f2c_route, planner);
                        } else if (config.path_planner == "reeds") {
                            f2c::pp::ReedsSheppCurves planner;
                            f2c_path = pp.planPath(robot, f2c_route, planner);
                        } else if (config.path_planner == "reeds_hc") {
                            f2c::pp::ReedsSheppCurvesHC planner;
                            f2c_path = pp.planPath(robot, f2c_route, planner);
                        } else {
                            // Default to Dubins
                            f2c::pp::DubinsCurves planner;
                            f2c_path = pp.planPath(robot, f2c_route, planner);
                        }
                        
                        // Extract path states from F2C path
                        for (size_t i = 0; i < f2c_path.size(); ++i) {
                            auto state = f2c_path.getState(i);
                            PathState ps;
                            ps.point = Point2D(state.point.getX(), state.point.getY());
                            ps.heading = state.angle;
                            ps.vx = std::cos(ps.heading);
                            ps.vy = std::sin(ps.heading);
                            result.path.push_back(ps);
                        }
                        
                        // IMPORTANT: Ensure path ends at the last route waypoint
                        // Smooth curve planners may not sample exactly at endpoints
                        if (!result.path.empty() && !result.route.empty()) {
                            const auto& last_route = result.route.back();
                            const auto& last_path = result.path.back();
                            double dx = last_route.point.x - last_path.point.x;
                            double dy = last_route.point.y - last_path.point.y;
                            double dist = std::sqrt(dx*dx + dy*dy);
                            
                            // If last path point differs from last route point by > 1cm, append it
                            if (dist > 0.01) {
                                PathState end_state;
                                end_state.point = last_route.point;
                                // Use heading from last path point
                                end_state.heading = last_path.heading;
                                end_state.vx = std::cos(end_state.heading);
                                end_state.vy = std::sin(end_state.heading);
                                result.path.push_back(end_state);
                            }
                        }
                        
                    } catch (const std::exception& e) {
                        std::cerr << "Path planning warning: " << e.what() << std::endl;
                        result.path = swathsToAxialTurnPath(result.swaths);
                    }
                }
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Route generation warning: " << e.what() << std::endl;
        }
        
        reportProgress(100, "Coverage generation complete");
        result.success = true;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = e.what();
    }
    
    return result;
}

#else // No Fields2Cover

CoverageResult generateCoverage(const Polygon2D& boundary,
                                const CoverageConfig& config,
                                const Polygon2D* roi,
                                const std::vector<Polygon2D>* obstacles) {
    CoverageResult result;
    
    // Note: obstacles are not handled in the simple fallback version
    (void)obstacles;
    
    reportProgress(5, "Generating simple coverage (no F2C)...");
    
    // Simple coverage without F2C - generate parallel swaths
    const Polygon2D& effective_poly = (roi && !roi->empty()) ? *roi : boundary;
    
    double min_x, min_y, max_x, max_y;
    polygonBounds(effective_poly, min_x, min_y, max_x, max_y);
    
    // Add headland
    min_x += config.headland_width;
    min_y += config.headland_width;
    max_x -= config.headland_width;
    max_y -= config.headland_width;
    
    if (min_x >= max_x || min_y >= max_y) {
        result.error_message = "Area too small after headland";
        return result;
    }
    
    // Compute direction
    double direction = M_PI / 2.0;  // Default: vertical
    if (config.auto_align) {
        double base_angle = computeSwathAngle(effective_poly);
        direction = (config.align_mode == "long") ? base_angle : base_angle + M_PI / 2.0;
    }
    
    // Generate swaths along direction
    reportProgress(30, "Generating swaths...");
    
    double cos_d = std::cos(direction);
    double sin_d = std::sin(direction);
    double perp_x = -sin_d;
    double perp_y = cos_d;
    
    // Project bounds onto direction axis
    double center_x = (min_x + max_x) / 2;
    double center_y = (min_y + max_y) / 2;
    double width = std::max(max_x - min_x, max_y - min_y);
    
    int num_swaths = static_cast<int>(width / config.swath_width) + 1;
    double start_offset = -width / 2;
    
    for (int i = 0; i < num_swaths; ++i) {
        double offset = start_offset + i * config.swath_width;
        
        double base_x = center_x + offset * perp_x;
        double base_y = center_y + offset * perp_y;
        
        Swath swath;
        swath.start = Point2D(base_x - width * cos_d, base_y - width * sin_d);
        swath.end = Point2D(base_x + width * cos_d, base_y + width * sin_d);
        swath.heading = direction;
        
        // Reverse direction for boustrophedon pattern
        if (i % 2 == 1) {
            std::swap(swath.start, swath.end);
            swath.heading = direction + M_PI;
        }
        
        result.swaths.push_back(swath);
    }
    
    // Generate path from swaths
    reportProgress(70, "Generating path...");
    result.path = swathsToAxialTurnPath(result.swaths);
    
    // Route is same as path endpoints
    for (const auto& sw : result.swaths) {
        PathState s;
        s.point = sw.start;
        s.heading = sw.heading;
        result.route.push_back(s);
        
        PathState e;
        e.point = sw.end;
        e.heading = sw.heading;
        result.route.push_back(e);
    }
    
    reportProgress(100, "Coverage complete");
    result.success = true;
    
    return result;
}

#endif // HAVE_FIELDS2COVER

// =============================================================================
// Export Functions
// =============================================================================

bool savePathToCSV(const PathStateList& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << "x,y\n";
    for (const auto& state : path) {
        file << std::fixed << std::setprecision(6) 
             << state.point.x << "," << state.point.y << "\n";
    }
    
    return true;
}

} // namespace f2c_cpp
