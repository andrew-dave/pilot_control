#ifndef PILOT_CONTROL_ROOF_EDGE_GRID_PLANNER_HPP
#define PILOT_CONTROL_ROOF_EDGE_GRID_PLANNER_HPP

#include <cstdint>
#include <utility>
#include <vector>

namespace pilot_control {

// Minimal 2D point, decoupled from the OCU-side coverage_geometry types so the
// planner ports cleanly into the robot package with zero Qt / F2C dependencies.
struct Point2D {
    double x = 0.0;
    double y = 0.0;
    Point2D() = default;
    Point2D(double x_in, double y_in) : x(x_in), y(y_in) {}
};

// Jump Point Search planner over an inflated occupancy grid.
//
// This is a faithful port of the OCU-side f2c_cpp::GridPlanner JPS core (EDT
// inflation + JPS + any-angle shortcut smoothing). The polygon-ingest build()
// path is intentionally dropped; the roof-edge costmap feeds an occupancy grid
// directly via buildFromGrid(), which is cheaper than rasterising polygons and
// matches the cell-classification output 1:1.
class RoofEdgeGridPlanner {
public:
    // Build the planner from a tiered occupancy grid (all vectors indexed
    // idx(x,y), size width*height):
    //   lethal_lo — obstacle cells inflated by `inflation_lo` metres.
    //   lethal_hi — obstacle cells inflated by `inflation_hi` metres (confirmed
    //               prominent clusters + cliffs); typically a subset of lethal_lo.
    //   blocked   — cells that are non-traversable but never inflate (UNKNOWN
    //               and de-noised speckle). A blocked cell blocks only itself.
    // A cell becomes lethal when it lies within `inflation_lo` of any lethal_lo
    // seed, OR within `inflation_hi` of any lethal_hi seed, OR is blocked.
    // Returns false on degenerate geometry.
    bool buildFromGrid(const std::vector<uint8_t>& lethal_lo,
                       const std::vector<uint8_t>& lethal_hi,
                       const std::vector<uint8_t>& blocked, int width, int height,
                       double origin_x, double origin_y, double resolution,
                       double inflation_lo, double inflation_hi);

    // Plan from `from` to `to` in world (grid) coordinates. Returns a polyline
    // including endpoints, or empty if no path exists. Endpoints are snapped to
    // the nearest free cell within `snap_radius_m`.
    std::vector<Point2D> plan(const Point2D& from, const Point2D& to,
                              double snap_radius_m = 0.5,
                              bool bias_clearance = false) const;

    bool valid() const { return !lethal_.empty(); }
    bool lineOfSight(const Point2D& a, const Point2D& b) const;
    float clearanceAtWorld(const Point2D& p) const;

    int width() const { return w_; }
    int height() const { return h_; }
    double resolution() const { return res_; }

private:
    static constexpr long long kMaxCells = 4'000'000;

    int idx(int x, int y) const { return y * w_ + x; }
    bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    bool freeCell(int x, int y) const { return inBounds(x, y) && lethal_[idx(x, y)] == 0; }

    bool worldToCell(const Point2D& p, int& cx, int& cy) const;
    Point2D cellToWorld(int cx, int cy) const;
    float clearanceAt(int cx, int cy) const;
    bool snapToFree(int& cx, int& cy, double radius_m) const;

    void computeEdt(const std::vector<uint8_t>& occupied, std::vector<float>& clearance) const;

    bool losCells(int x0, int y0, int x1, int y1) const;
    int jump(int x, int y, int dx, int dy, int gx, int gy) const;
    std::vector<std::pair<int, int>> prunedDirs(int cur, int parent) const;
    std::vector<std::pair<int, int>> jpsCells(int sx, int sy, int gx, int gy) const;
    std::vector<Point2D> smooth(const std::vector<Point2D>& pts, bool bias_clearance) const;

    int w_ = 0;
    int h_ = 0;
    double res_ = 0.05;
    double ox_ = 0.0;
    double oy_ = 0.0;
    double inflation_ = 0.0;
    std::vector<uint8_t> lethal_;
    std::vector<float> clearance_;
};

}  // namespace pilot_control

#endif  // PILOT_CONTROL_ROOF_EDGE_GRID_PLANNER_HPP
