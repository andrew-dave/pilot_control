/**
 * @file coverage_gui.cpp
 * @brief Qt6 GUI implementation for coverage planning
 */

#include "coverage_gui.hpp"

#include <QApplication>
#include <QStyle>
#include <QFont>
#include <QPen>
#include <QBrush>
#include <QToolTip>
#include <QFileInfo>
#include <QFile>
#include <QFrame>
#include <QSignalBlocker>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdlib>

// PCL for 3D point cloud preview
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace f2c_cpp {

namespace {

constexpr double kWaypointDuplicateEpsilon = 1e-6;

PathStateList dedupePathStates(const PathStateList& path) {
    PathStateList filtered;
    filtered.reserve(path.size());
    for (const auto& state : path) {
        if (!filtered.empty()) {
            double dx = state.point.x - filtered.back().point.x;
            double dy = state.point.y - filtered.back().point.y;
            if (std::fabs(dx) <= kWaypointDuplicateEpsilon &&
                std::fabs(dy) <= kWaypointDuplicateEpsilon) {
                continue;
            }
        }
        filtered.push_back(state);
    }
    return filtered;
}

} // namespace

// =============================================================================
// PlotWidget Implementation
// =============================================================================

PlotWidget::PlotWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 400);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    
    // White background
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
}

void PlotWidget::setPoints(const std::vector<Point2D>& points) {
    points_ = points;
    updateDataBounds();
    update();
}

void PlotWidget::setPolygon(const Polygon2D& poly) {
    polygon_ = poly;
    updateDataBounds();
    update();
}

void PlotWidget::setROI(const Polygon2D& roi) {
    roi_ = roi;
    update();
}

void PlotWidget::setObstacles(const std::vector<Polygon2D>& obstacles) {
    obstacles_ = obstacles;
    update();
}

void PlotWidget::setSwaths(const SwathList& swaths) {
    swaths_ = swaths;
    update();
}

void PlotWidget::setRoute(const PathStateList& route) {
    route_ = route;
    update();
}

void PlotWidget::setPath(const PathStateList& path) {
    path_ = path;
    update();
}

void PlotWidget::setRobotPose(const std::optional<PathState>& pose) {
    robot_pose_ = pose;
    update();
}

void PlotWidget::setRobotTrail(const std::vector<Point2D>& trail) {
    robot_trail_ = trail;
    update();
}

void PlotWidget::setRobotMarkerSize(double size_meters) {
    robot_marker_size_ = std::max(0.05, size_meters);
    update();
}

void PlotWidget::setCustomPath(const std::vector<Point2D>& path,
                               const std::vector<bool>& visited) {
    custom_waypoints_ = path;
    custom_waypoint_states_ = visited;
    if (custom_waypoint_states_.size() < custom_waypoints_.size()) {
        custom_waypoint_states_.resize(custom_waypoints_.size(), false);
    }
    update();
}

void PlotWidget::setShowCustomPath(bool show) {
    show_custom_path_ = show;
    update();
}

void PlotWidget::setCustomDrawMode(bool enabled) {
    custom_draw_mode_ = enabled;
}

void PlotWidget::setReprojectionLines(const std::vector<ReprojectionLine>& lines) {
    reproj_lines_ = lines;
    hovered_reproj_index_ = -1;
    update();
}

void PlotWidget::clearReprojectionLines() {
    reproj_lines_.clear();
    hovered_reproj_index_ = -1;
    update();
}

double PlotWidget::distanceToLineSegment(const QPointF& mouse, const QPointF& p1, const QPointF& p2) const {
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double len_sq = dx * dx + dy * dy;
    
    if (len_sq < 1e-10) {
        // Degenerate line (points are the same)
        return std::hypot(mouse.x() - p1.x(), mouse.y() - p1.y());
    }
    
    // Project mouse onto line, clamped to segment
    double t = std::max(0.0, std::min(1.0, 
        ((mouse.x() - p1.x()) * dx + (mouse.y() - p1.y()) * dy) / len_sq));
    
    double proj_x = p1.x() + t * dx;
    double proj_y = p1.y() + t * dy;
    
    return std::hypot(mouse.x() - proj_x, mouse.y() - proj_y);
}

void PlotWidget::clearAll() {
    points_.clear();
    polygon_.clear();
    roi_.clear();
    obstacles_.clear();
    swaths_.clear();
    route_.clear();
    path_.clear();
    robot_pose_.reset();
    robot_trail_.clear();
    custom_waypoints_.clear();
    custom_waypoint_states_.clear();
    reproj_lines_.clear();
    hovered_reproj_index_ = -1;
    selection_points_.clear();
    selecting_ = false;
    update();
}

void PlotWidget::clearPoints() { points_.clear(); update(); }
void PlotWidget::clearPolygon() { polygon_.clear(); update(); }
void PlotWidget::clearROI() { roi_.clear(); update(); }
void PlotWidget::clearObstacles() { obstacles_.clear(); update(); }
void PlotWidget::clearSwaths() { swaths_.clear(); update(); }
void PlotWidget::clearRoute() { route_.clear(); update(); }
void PlotWidget::clearPath() { path_.clear(); update(); }

void PlotWidget::resetView() {
    updateDataBounds();
    fitToData();
    update();
}

void PlotWidget::zoomIn() {
    scale_ *= 1.2;
    update();
}

void PlotWidget::zoomOut() {
    scale_ /= 1.2;
    update();
}

void PlotWidget::startROISelection() {
    selecting_ = true;
    selecting_roi_ = true;
    selection_points_.clear();
    setCursor(Qt::CrossCursor);
    update();
}

void PlotWidget::startObstacleSelection() {
    selecting_ = true;
    selecting_roi_ = false;
    selection_points_.clear();
    setCursor(Qt::CrossCursor);
    update();
}

void PlotWidget::finishSelection() {
    if (!selecting_ || selection_points_.size() < 3) {
        cancelSelection();
        return;
    }
    
    selecting_ = false;
    setCursor(Qt::ArrowCursor);
    
    Polygon2D poly = selection_points_;
    selection_points_.clear();
    
    if (selecting_roi_) {
        emit roiSelected(poly);
    } else {
        emit obstacleSelected(poly);
    }
    
    update();
}

void PlotWidget::cancelSelection() {
    selecting_ = false;
    selection_points_.clear();
    setCursor(Qt::ArrowCursor);
    emit selectionCancelled();
    update();
}

void PlotWidget::undoLastPoint() {
    if (!selection_points_.empty()) {
        selection_points_.pop_back();
        update();
    }
}

Polygon2D PlotWidget::getSelectedPolygon() const {
    return selection_points_;
}

void PlotWidget::updateDataBounds() {
    data_min_x_ = data_min_y_ = std::numeric_limits<double>::max();
    data_max_x_ = data_max_y_ = std::numeric_limits<double>::lowest();
    
    auto updateBounds = [&](const Point2D& p) {
        data_min_x_ = std::min(data_min_x_, p.x);
        data_max_x_ = std::max(data_max_x_, p.x);
        data_min_y_ = std::min(data_min_y_, p.y);
        data_max_y_ = std::max(data_max_y_, p.y);
    };
    
    for (const auto& p : points_) updateBounds(p);
    for (const auto& p : polygon_) updateBounds(p);
    for (const auto& p : roi_) updateBounds(p);
    for (const auto& obs : obstacles_) {
        for (const auto& p : obs) updateBounds(p);
    }
    for (const auto& sw : swaths_) {
        updateBounds(sw.start);
        updateBounds(sw.end);
    }
    for (const auto& st : path_) updateBounds(st.point);
    if (robot_pose_.has_value()) {
        updateBounds(robot_pose_->point);
    }
    for (const auto& trail_pt : robot_trail_) {
        updateBounds(trail_pt);
    }
    for (const auto& wp : custom_waypoints_) {
        updateBounds(wp);
    }
    
    if (data_min_x_ > data_max_x_) {
        data_min_x_ = 0; data_max_x_ = 1;
        data_min_y_ = 0; data_max_y_ = 1;
    }
    
    // Add margin
    double margin_x = (data_max_x_ - data_min_x_) * 0.05;
    double margin_y = (data_max_y_ - data_min_y_) * 0.05;
    data_min_x_ -= margin_x;
    data_max_x_ += margin_x;
    data_min_y_ -= margin_y;
    data_max_y_ += margin_y;
}

void PlotWidget::fitToData() {
    updateDataBounds();
    
    double data_w = data_max_x_ - data_min_x_;
    double data_h = data_max_y_ - data_min_y_;
    
    if (data_w < 1e-10) data_w = 1;
    if (data_h < 1e-10) data_h = 1;
    
    double scale_x = (width() - 40) / data_w;
    double scale_y = (height() - 40) / data_h;
    scale_ = std::min(scale_x, scale_y);
    
    double center_x = (data_min_x_ + data_max_x_) / 2;
    double center_y = (data_min_y_ + data_max_y_) / 2;
    
    offset_x_ = width() / 2 - center_x * scale_;
    offset_y_ = height() / 2 + center_y * scale_;  // Y flipped
}

QPointF PlotWidget::worldToScreen(const Point2D& p) const {
    return QPointF(p.x * scale_ + offset_x_, -p.y * scale_ + offset_y_);
}

Point2D PlotWidget::screenToWorld(const QPointF& p) const {
    return Point2D((p.x() - offset_x_) / scale_, -(p.y() - offset_y_) / scale_);
}

void PlotWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // Background
    painter.fillRect(rect(), Qt::white);
    
    // Draw grid
    painter.setPen(QPen(QColor(200, 200, 200), 1, Qt::DashLine));
    double grid_step = std::pow(10, std::floor(std::log10(std::max(data_max_x_ - data_min_x_, data_max_y_ - data_min_y_) / 5)));
    for (double x = std::floor(data_min_x_ / grid_step) * grid_step; x <= data_max_x_; x += grid_step) {
        QPointF p1 = worldToScreen(Point2D(x, data_min_y_));
        QPointF p2 = worldToScreen(Point2D(x, data_max_y_));
        painter.drawLine(p1, p2);
    }
    for (double y = std::floor(data_min_y_ / grid_step) * grid_step; y <= data_max_y_; y += grid_step) {
        QPointF p1 = worldToScreen(Point2D(data_min_x_, y));
        QPointF p2 = worldToScreen(Point2D(data_max_x_, y));
        painter.drawLine(p1, p2);
    }
    
    // Draw points
    if (!points_.empty()) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(100, 100, 100, 150));
        for (const auto& p : points_) {
            QPointF sp = worldToScreen(p);
            painter.drawEllipse(sp, 2, 2);  // Larger points for visibility
        }
    }
    
    // Draw polygon
    if (!polygon_.empty()) {
        painter.setPen(QPen(Qt::black, 2));
        painter.setBrush(Qt::NoBrush);
        QPolygonF poly_qp;
        for (const auto& p : polygon_) {
            poly_qp << worldToScreen(p);
        }
        poly_qp << poly_qp.first();  // Close polygon
        painter.drawPolygon(poly_qp);
    }
    
    // Draw ROI
    if (!roi_.empty()) {
        painter.setPen(QPen(Qt::green, 2));
        painter.setBrush(Qt::NoBrush);
        QPolygonF roi_qp;
        for (const auto& p : roi_) {
            roi_qp << worldToScreen(p);
        }
        roi_qp << roi_qp.first();
        painter.drawPolygon(roi_qp);
    }
    
    // Draw obstacles
    for (const auto& obs : obstacles_) {
        painter.setPen(QPen(Qt::red, 2));
        painter.setBrush(QColor(255, 0, 0, 50));
        QPolygonF obs_qp;
        for (const auto& p : obs) {
            obs_qp << worldToScreen(p);
        }
        obs_qp << obs_qp.first();
        painter.drawPolygon(obs_qp);
    }
    
    // Draw swaths
    if (!swaths_.empty()) {
        painter.setPen(QPen(Qt::blue, 1, Qt::DashLine));
        for (const auto& sw : swaths_) {
            QPointF p1 = worldToScreen(sw.start);
            QPointF p2 = worldToScreen(sw.end);
            painter.drawLine(p1, p2);
        }
    }
    
    // Draw route
    if (!route_.empty()) {
        painter.setPen(QPen(QColor(255, 140, 0), 1.5));
        for (size_t i = 1; i < route_.size(); ++i) {
            QPointF p1 = worldToScreen(route_[i-1].point);
            QPointF p2 = worldToScreen(route_[i].point);
            painter.drawLine(p1, p2);
        }
    }
    
    // Draw path
    if (!path_.empty()) {
        painter.setPen(QPen(Qt::red, 1.5));
        for (size_t i = 1; i < path_.size(); ++i) {
            QPointF p1 = worldToScreen(path_[i-1].point);
            QPointF p2 = worldToScreen(path_[i].point);
            painter.drawLine(p1, p2);
        }
        
        // Start and end markers
        if (!path_.empty()) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(Qt::green);
            QPointF start = worldToScreen(path_.front().point);
            painter.drawEllipse(start, 6, 6);
            
            painter.setBrush(Qt::red);
            QPointF end = worldToScreen(path_.back().point);
            painter.drawRect(QRectF(end.x() - 5, end.y() - 5, 10, 10));
        }
    }
    
    // Draw custom waypoint path
    if (show_custom_path_ && !custom_waypoints_.empty()) {
        painter.setPen(QPen(QColor(0, 150, 136), 2, Qt::SolidLine, Qt::RoundCap));
        for (size_t i = 1; i < custom_waypoints_.size(); ++i) {
            painter.drawLine(worldToScreen(custom_waypoints_[i-1]),
                             worldToScreen(custom_waypoints_[i]));
        }
        
        painter.setFont(QFont("Sans Serif", 8));
        for (size_t i = 0; i < custom_waypoints_.size(); ++i) {
            bool visited = (i < custom_waypoint_states_.size()) && custom_waypoint_states_[i];
            painter.setPen(QPen(Qt::black, 1));
            painter.setBrush(visited ? QColor(76, 175, 80) : QColor(0, 188, 212));
            QPointF pt = worldToScreen(custom_waypoints_[i]);
            painter.drawEllipse(pt, 5, 5);
            
            QString label = QString("#%1 (%2, %3)")
                .arg(i + 1)
                .arg(custom_waypoints_[i].x, 0, 'f', 2)
                .arg(custom_waypoints_[i].y, 0, 'f', 2);
            painter.drawText(pt + QPointF(8, -6), label);
        }
    }
    
    // Draw robot trail (live position history)
    if (!robot_trail_.empty()) {
        painter.setPen(QPen(QColor(30, 144, 255, 180), 2, Qt::SolidLine, Qt::RoundCap));
        for (size_t i = 1; i < robot_trail_.size(); ++i) {
            painter.drawLine(worldToScreen(robot_trail_[i-1]),
                             worldToScreen(robot_trail_[i]));
        }
    }
    
    if (robot_pose_.has_value()) {
        const auto& pose = robot_pose_.value();
        const double base = robot_marker_size_;
        const double wing = base * 0.6;
        
        Point2D tip(
            pose.point.x + base * std::cos(pose.heading),
            pose.point.y + base * std::sin(pose.heading));
        Point2D left(
            pose.point.x + wing * std::cos(pose.heading + 2.5),
            pose.point.y + wing * std::sin(pose.heading + 2.5));
        Point2D right(
            pose.point.x + wing * std::cos(pose.heading - 2.5),
            pose.point.y + wing * std::sin(pose.heading - 2.5));
        
        QPolygonF tri;
        tri << worldToScreen(tip)
            << worldToScreen(left)
            << worldToScreen(right);
        
        painter.setPen(QPen(QColor(128, 0, 128), 2));
        painter.setBrush(QColor(255, 192, 203, 230));
        painter.drawPolygon(tri);
        
        QPointF center = worldToScreen(pose.point);
        painter.setPen(QPen(Qt::black, 1));
        painter.setFont(QFont("Sans Serif", 8, QFont::Bold));
        painter.drawText(center + QPointF(8, -8), "Robot");
    }
    
    // Draw origin marker (robot position at 0,0)
    {
        QPointF origin = worldToScreen(Point2D(0.0, 0.0));
        
        // Check if origin is within view bounds (with some margin)
        if (origin.x() > -50 && origin.x() < width() + 50 &&
            origin.y() > -50 && origin.y() < height() + 50) {
            
            // Draw crosshairs
            painter.setPen(QPen(QColor(220, 20, 60), 2));  // Crimson color
            painter.drawLine(origin.x() - 18, origin.y(), origin.x() + 18, origin.y());  // Horizontal
            painter.drawLine(origin.x(), origin.y() - 18, origin.x(), origin.y() + 18);  // Vertical
            
            // Draw circle around origin
            painter.setPen(QPen(QColor(220, 20, 60), 2));
            painter.setBrush(QColor(220, 20, 60, 40));  // Semi-transparent fill
            painter.drawEllipse(origin, 12, 12);
            
            // Draw inner dot
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(220, 20, 60));
            painter.drawEllipse(origin, 3, 3);
            
            // Draw label
            painter.setPen(QColor(220, 20, 60));
            painter.setFont(QFont("Sans Serif", 9, QFont::Bold));
            painter.drawText(origin.x() + 16, origin.y() - 8, "Origin");
            painter.setFont(QFont("Sans Serif", 8));
            painter.drawText(origin.x() + 16, origin.y() + 6, "(0, 0)");
        }
    }
    
    // Draw reprojection error lines
    if (!reproj_lines_.empty()) {
        for (size_t i = 0; i < reproj_lines_.size(); ++i) {
            const auto& line = reproj_lines_[i];
            QPointF wp_screen = worldToScreen(line.waypoint);
            QPointF tr_screen = worldToScreen(line.traversed);
            
            bool is_hovered = (static_cast<int>(i) == hovered_reproj_index_);
            
            // Draw line: red normally, green when hovered
            QPen pen(is_hovered ? QColor(0, 200, 0) : QColor(220, 50, 50));
            pen.setWidth(is_hovered ? 3 : 2);
            painter.setPen(pen);
            painter.drawLine(wp_screen, tr_screen);
            
            // Draw small circles at endpoints
            painter.setBrush(is_hovered ? QColor(0, 200, 0) : QColor(220, 50, 50));
            painter.drawEllipse(wp_screen, 4, 4);
            painter.drawEllipse(tr_screen, 4, 4);
            
            // If hovered, also show error text near the line
            if (is_hovered) {
                double error_cm = line.error_m * 100.0;
                QPointF mid((wp_screen.x() + tr_screen.x()) / 2,
                           (wp_screen.y() + tr_screen.y()) / 2);
                painter.setPen(Qt::white);
                painter.setFont(QFont("Arial", 10, QFont::Bold));
                
                // Draw background for readability
                QString text = QString("%1 cm").arg(error_cm, 0, 'f', 1);
                QRectF textRect = painter.fontMetrics().boundingRect(text);
                textRect.moveCenter(mid);
                textRect.adjust(-3, -2, 3, 2);
                painter.fillRect(textRect, QColor(0, 0, 0, 180));
                painter.drawText(textRect, Qt::AlignCenter, text);
            }
        }
    }
    
    // Draw selection in progress
    if (selecting_ && !selection_points_.empty()) {
        painter.setPen(QPen(Qt::magenta, 1.5));
        painter.setBrush(QColor(255, 0, 255, 30));
        
        QPolygonF sel_qp;
        for (const auto& p : selection_points_) {
            sel_qp << worldToScreen(p);
        }
        
        // Draw preview line to cursor
        if (cursor_pos_ != QPointF()) {
            sel_qp << cursor_pos_;
        }
        
        if (selection_points_.size() >= 3) {
            sel_qp << sel_qp.first();  // Close for fill
            painter.drawPolygon(sel_qp);
        } else if (selection_points_.size() >= 2) {
            painter.setBrush(Qt::NoBrush);
            painter.drawPolyline(sel_qp);
        }
        
        // Draw points
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::magenta);
        for (const auto& p : selection_points_) {
            QPointF sp = worldToScreen(p);
            painter.drawEllipse(sp, 4, 4);
        }
    }
    
    // Draw title
    painter.setPen(Qt::black);
    painter.setFont(QFont("Sans Serif", 10, QFont::Bold));
    painter.drawText(10, 20, "2D Projection / Coverage");
}

void PlotWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        if (custom_draw_mode_) {
            Point2D world = screenToWorld(event->pos());
            emit customWaypointRequested(world);
            return;
        }
        
        if (selecting_) {
            Point2D world = screenToWorld(event->pos());
            selection_points_.push_back(world);
            update();
        } else {
            // Start panning
            panning_ = true;
            pan_start_ = event->pos();
            pan_offset_x_ = offset_x_;
            pan_offset_y_ = offset_y_;
            setCursor(Qt::ClosedHandCursor);
        }
    } else if (event->button() == Qt::RightButton) {
        if (selecting_) {
            finishSelection();
        }
    } else if (event->button() == Qt::MiddleButton) {
        resetView();
    }
}

void PlotWidget::mouseMoveEvent(QMouseEvent* event) {
    cursor_pos_ = event->pos();
    
    if (panning_) {
        offset_x_ = pan_offset_x_ + (event->pos().x() - pan_start_.x());
        offset_y_ = pan_offset_y_ + (event->pos().y() - pan_start_.y());
        update();
    } else if (selecting_) {
        update();  // Redraw preview line
    }
    
    // Check hover on reprojection lines
    if (!reproj_lines_.empty()) {
        int old_hovered = hovered_reproj_index_;
        hovered_reproj_index_ = -1;
        
        const double hover_threshold = 8.0;  // pixels
        double min_dist = hover_threshold;
        
        for (size_t i = 0; i < reproj_lines_.size(); ++i) {
            QPointF wp_screen = worldToScreen(reproj_lines_[i].waypoint);
            QPointF tr_screen = worldToScreen(reproj_lines_[i].traversed);
            double dist = distanceToLineSegment(event->pos(), wp_screen, tr_screen);
            
            if (dist < min_dist) {
                min_dist = dist;
                hovered_reproj_index_ = static_cast<int>(i);
            }
        }
        
        if (hovered_reproj_index_ != old_hovered) {
            update();
            
            // Show reprojection tooltip if hovering a line
            if (hovered_reproj_index_ >= 0) {
                const auto& line = reproj_lines_[hovered_reproj_index_];
                double error_cm = line.error_m * 100.0;
                QString tip = QString("WP %1: %2 cm error")
                    .arg(line.waypoint_index + 1)
                    .arg(error_cm, 0, 'f', 1);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
                QToolTip::showText(event->globalPosition().toPoint(), tip, this);
#else
                QToolTip::showText(event->globalPos(), tip, this);
#endif
                return;  // Don't show coordinate tooltip
            }
        }
        
        // If still hovering a reprojection line, keep showing its tooltip
        if (hovered_reproj_index_ >= 0) {
            return;
        }
    }
    
    // Show coordinates in tooltip
    Point2D world = screenToWorld(event->pos());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QToolTip::showText(event->globalPosition().toPoint(), 
                      QString("(%1, %2)").arg(world.x, 0, 'f', 2).arg(world.y, 0, 'f', 2));
#else
    QToolTip::showText(event->globalPos(), 
                      QString("(%1, %2)").arg(world.x, 0, 'f', 2).arg(world.y, 0, 'f', 2));
#endif
}

void PlotWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        if (panning_) {
            panning_ = false;
            setCursor(Qt::ArrowCursor);
        }
    }
}

void PlotWidget::wheelEvent(QWheelEvent* event) {
    double factor = (event->angleDelta().y() > 0) ? 1.2 : (1.0 / 1.2);
    
    // Zoom centered on cursor
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    QPointF cursor = event->position();
#else
    QPointF cursor = event->posF();
#endif
    Point2D world_before = screenToWorld(cursor);
    
    scale_ *= factor;
    
    // Adjust offset to keep cursor position fixed
    offset_x_ = cursor.x() - world_before.x * scale_;
    offset_y_ = cursor.y() + world_before.y * scale_;
    
    update();
}

void PlotWidget::resizeEvent(QResizeEvent* event) {
    Q_UNUSED(event);
    // Optionally fit to data on resize
}

// =============================================================================
// CoverageGUI Implementation
// =============================================================================

CoverageGUI::CoverageGUI(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("Roof Coverage Planner (C++)");
    resize(1500, 900);
    
    // Initialize robot map fetch settings (user-agnostic)
    local_map_base_ = QDir::homePath() + "/Roofus_maps";
    
    // Initialize CycloneDDS config paths (user-agnostic)
    dds_rf_config_path_ = QDir::homePath() + "/rf_cyclonedds.xml";
    dds_wifi_config_path_ = QDir::homePath() + "/wifi_cyclonedds.xml";

    // Load persisted settings
    QSettings settings("PilotControl", "F2CCoveragePlanner");
    robot_host_ = settings.value("robot_ip", robot_host_).toString();
    dds_profile_ = settings.value("dds_profile", "rf").toString();  // Default to RF
    robot_odom_topic_ = settings.value("robot_odom_topic", robot_odom_topic_).toString();
    robot_marker_size_m_ = settings.value("robot_marker_size_m", robot_marker_size_m_).toDouble();
    
    // Set CYCLONEDDS_URI environment variable based on saved profile
    QString dds_config = currentDdsConfigPath();
    if (QFile::exists(dds_config)) {
        qputenv("CYCLONEDDS_URI", dds_config.toUtf8());
        std::cout << "[F2C GUI] Using CycloneDDS config: " << dds_config.toStdString() << std::endl;
    } else {
        std::cerr << "[F2C GUI] Warning: DDS config not found: " << dds_config.toStdString() << std::endl;
    }
    
    fit_view_pending_ = true;
    setupUI();
    setupConnections();
    refreshCustomPathUI();

    // Initialize ROS2 reconnection timer (will only run when disconnected)
    ros_reconnect_timer_ = new QTimer(this);
    ros_reconnect_timer_->setInterval(5000);  // Try every 5 seconds
    connect(ros_reconnect_timer_, &QTimer::timeout, this, &CoverageGUI::tryReconnectROS2);

    // Initialize ROS2 (with error handling so GUI works even if ROS2 fails)
    waypoints_published_ = false;
    ros_initialized_ = false;
    try {
        ros_node_ = rclcpp::Node::make_shared("f2c_coverage_gui");
        waypoint_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/f2c_waypoints", 10);
        setupRobotTrackingSubscription();

        // Start ROS2 spinning in background thread
        ros_thread_ = std::thread([this]() {
            rclcpp::spin(ros_node_);
        });
        ros_initialized_ = true;
        setStatus(QString("Ready (ROS2 connected via %1)").arg(dds_profile_.toUpper()));
    } catch (const std::exception& e) {
        std::cerr << "[F2C GUI] Warning: ROS2 initialization failed: " << e.what() << std::endl;
        std::cerr << "[F2C GUI] Starting background reconnection timer..." << std::endl;
        setStatus("Ready (ROS2 unavailable - reconnecting...)");
        
        // Start the reconnection timer
        ros_reconnect_timer_->start();
    }
}

CoverageGUI::~CoverageGUI() {
    // Stop reconnection timer
    if (ros_reconnect_timer_) {
        ros_reconnect_timer_->stop();
    }
    
    // Clean up ROS2 resources
    fastlio_sub_.reset();
    waypoint_pub_.reset();
    
    if (ros_node_) {
        rclcpp::shutdown();
    }
    
    // Wait for ROS thread to finish
    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
    
    ros_node_.reset();
}

void CoverageGUI::setupUI() {
    QWidget* central = new QWidget();
    QHBoxLayout* main_layout = new QHBoxLayout(central);
    
    // Use regular horizontal layout instead of splitter for fixed panel width
    // Controls panel - fixed width
    QScrollArea* controls_scroll = new QScrollArea();
    controls_scroll->setWidgetResizable(true);
    controls_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controls_scroll->setFixedWidth(380);  // Fixed width - won't resize
    
    QWidget* controls_container = new QWidget();
    QVBoxLayout* controls_layout = new QVBoxLayout(controls_container);
    controls_layout->setContentsMargins(12, 12, 12, 12);
    controls_layout->setSpacing(12);
    
    // Single panel layout (no tabs)
    controls_layout->addWidget(buildFileControls());
    controls_layout->addWidget(buildRobotTrackingControls());
    controls_layout->addWidget(buildHeightControls());
    controls_layout->addWidget(buildDownsampleControls());
    controls_layout->addWidget(buildHullControls());
    controls_layout->addWidget(buildSimplifyControls());
    controls_layout->addWidget(buildPathPlanningControls());
    controls_layout->addWidget(buildExportControls());
    controls_layout->addStretch(1);
    
    controls_scroll->setWidget(controls_container);
    main_layout->addWidget(controls_scroll);
    
    // Plot panel - takes remaining space
    QWidget* plot_container = new QWidget();
    QVBoxLayout* plot_layout = new QVBoxLayout(plot_container);
    plot_layout->setContentsMargins(0, 0, 0, 0);
    
    plot_ = new PlotWidget();
    plot_->setMinimumSize(400, 400);  // Minimum plot size
    plot_->setRobotMarkerSize(robot_marker_size_m_);
    plot_layout->addWidget(plot_, 1);
    
    // Toolbar for plot
    QHBoxLayout* toolbar = new QHBoxLayout();
    QPushButton* btn_reset_view = new QPushButton("Reset View");
    QPushButton* btn_zoom_in = new QPushButton("+");
    QPushButton* btn_zoom_out = new QPushButton("-");
    btn_zoom_in->setFixedWidth(30);
    btn_zoom_out->setFixedWidth(30);
    toolbar->addWidget(btn_reset_view);
    toolbar->addWidget(btn_zoom_in);
    toolbar->addWidget(btn_zoom_out);
    toolbar->addStretch();
    plot_layout->addLayout(toolbar);
    
    connect(btn_reset_view, &QPushButton::clicked, plot_, &PlotWidget::resetView);
    connect(btn_zoom_in, &QPushButton::clicked, plot_, &PlotWidget::zoomIn);
    connect(btn_zoom_out, &QPushButton::clicked, plot_, &PlotWidget::zoomOut);
    
    main_layout->addWidget(plot_container, 1);  // Stretch factor 1 - takes remaining space
    
    setCentralWidget(central);
    
    // Status bar
    status_bar_ = new QStatusBar();
    setStatusBar(status_bar_);
    
    progress_bar_ = new QProgressBar();
    progress_bar_->setVisible(false);
    progress_bar_->setMaximumWidth(200);
    status_bar_->addPermanentWidget(progress_bar_);
}

void CoverageGUI::setupConnections() {
    connect(plot_, &PlotWidget::roiSelected, this, &CoverageGUI::onROISelected);
    connect(plot_, &PlotWidget::obstacleSelected, this, &CoverageGUI::onObstacleSelected);
    connect(plot_, &PlotWidget::selectionCancelled, this, &CoverageGUI::onSelectionCancelled);
    connect(plot_, &PlotWidget::customWaypointRequested, this, &CoverageGUI::onPlotCustomWaypoint);
    
    // Path mode switching
    if (radio_mode_f2c_) {
        connect(radio_mode_f2c_, &QRadioButton::toggled, this, &CoverageGUI::onPathModeChanged);
    }
    
    if (btn_custom_draw_) {
        connect(btn_custom_draw_, &QPushButton::toggled, this, [this](bool checked) {
            custom_draw_enabled_ = checked;
            plot_->setCustomDrawMode(checked && isCustomModeActive());
            if (checked) {
                setStatus("Custom draw enabled - click on the map to add waypoints");
            }
        });
    }
    if (btn_custom_undo_) {
        connect(btn_custom_undo_, &QPushButton::clicked, this, &CoverageGUI::undoCustomWaypoint);
    }
    if (btn_custom_clear_) {
        connect(btn_custom_clear_, &QPushButton::clicked, this, &CoverageGUI::clearCustomWaypoints);
    }
    
    // Set progress callback
    setProgressCallback([this](int percent, const std::string& msg) {
        QMetaObject::invokeMethod(this, [this, percent, msg]() {
            updateProgress(percent, QString::fromStdString(msg));
        }, Qt::QueuedConnection);
    });
}

QGroupBox* CoverageGUI::buildFileControls() {
    QGroupBox* box = new QGroupBox("Point Cloud & Network");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    // CycloneDDS profile selection (RF vs WiFi)
    QHBoxLayout* dds_layout = new QHBoxLayout();
    dds_layout->addWidget(new QLabel("Network:"));
    radio_dds_rf_ = new QRadioButton("RF (Microhard)");
    radio_dds_wifi_ = new QRadioButton("WiFi");
    radio_dds_rf_->setToolTip("Use RF CycloneDDS config (~/" + QFileInfo(dds_rf_config_path_).fileName() + ")");
    radio_dds_wifi_->setToolTip("Use WiFi CycloneDDS config (~/" + QFileInfo(dds_wifi_config_path_).fileName() + ")");
    
    // Set initial selection based on persisted profile
    if (dds_profile_ == "wifi") {
        radio_dds_wifi_->setChecked(true);
    } else {
        radio_dds_rf_->setChecked(true);
    }
    
    dds_layout->addWidget(radio_dds_rf_);
    dds_layout->addWidget(radio_dds_wifi_);
    dds_layout->addStretch();
    v->addLayout(dds_layout);
    
    // DDS status label
    lbl_dds_status_ = new QLabel();
    lbl_dds_status_->setStyleSheet("color: #666; font-size: 10px;");
    QString config_path = currentDdsConfigPath();
    if (QFile::exists(config_path)) {
        lbl_dds_status_->setText("✓ Config: ~/" + QFileInfo(config_path).fileName());
        lbl_dds_status_->setStyleSheet("color: green; font-size: 10px;");
    } else {
        lbl_dds_status_->setText("⚠ Config not found: ~/" + QFileInfo(config_path).fileName());
        lbl_dds_status_->setStyleSheet("color: orange; font-size: 10px;");
    }
    v->addWidget(lbl_dds_status_);
    
    // Connect radio buttons to profile change handler
    connect(radio_dds_rf_, &QRadioButton::toggled, this, &CoverageGUI::onDdsProfileChanged);
    
    // Robot IP configuration
    QHBoxLayout* ip_layout = new QHBoxLayout();
    ip_layout->addWidget(new QLabel("Robot IP:"));
    txt_robot_ip_ = new QLineEdit(robot_host_);
    txt_robot_ip_->setPlaceholderText("e.g. 192.168.168.101");
    txt_robot_ip_->setToolTip("IP address for fetching maps via SSH");
    ip_layout->addWidget(txt_robot_ip_);
    v->addLayout(ip_layout);
    
    // Load from local file
    QPushButton* btn_load = new QPushButton("Load PCD / PLY / XYZ");
    btn_load->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
    connect(btn_load, &QPushButton::clicked, this, &CoverageGUI::loadPointCloud);
    v->addWidget(btn_load);
    
    // Fetch from robot via SSH
    QPushButton* btn_fetch = new QPushButton("📡 Fetch Latest from Robot");
    auto updateFetchTooltip = [this, btn_fetch]() {
        btn_fetch->setToolTip(
            QString("Download the latest map from robot (%1@%2)\nSaves to ~/Roofus_maps/")
            .arg(robot_user_, robot_host_));
    };
    updateFetchTooltip();
    btn_fetch->setStyleSheet("QPushButton { background-color: #e8f4f8; }");
    connect(btn_fetch, &QPushButton::clicked, this, &CoverageGUI::fetchLatestMapFromRobot);
    
    connect(txt_robot_ip_, &QLineEdit::editingFinished, this, [this, updateFetchTooltip]() mutable {
        QString trimmed = txt_robot_ip_->text().trimmed();
        if (trimmed != txt_robot_ip_->text()) {
            txt_robot_ip_->setText(trimmed);
        }
        robot_host_ = trimmed;
        QSettings settings("PilotControl", "F2CCoveragePlanner");
        settings.setValue("robot_ip", robot_host_);
        updateFetchTooltip();
    });
    v->addWidget(btn_fetch);
    
    lbl_file_ = new QLabel("No file loaded");
    v->addWidget(lbl_file_);
    
    return box;
}

QGroupBox* CoverageGUI::buildRobotTrackingControls() {
    QGroupBox* box = new QGroupBox("Robot Tracking");
    QVBoxLayout* layout = new QVBoxLayout(box);
    
    QHBoxLayout* topic_layout = new QHBoxLayout();
    topic_layout->addWidget(new QLabel("Odom topic:"));
    txt_robot_topic_ = new QLineEdit(robot_odom_topic_);
    txt_robot_topic_->setPlaceholderText("/Odometry_tilt_corrected_diff");
    topic_layout->addWidget(txt_robot_topic_);
    layout->addLayout(topic_layout);
    
    QHBoxLayout* size_layout = new QHBoxLayout();
    size_layout->addWidget(new QLabel("Marker size (m):"));
    spin_robot_marker_size_ = new QDoubleSpinBox();
    spin_robot_marker_size_->setRange(0.05, 3.0);
    spin_robot_marker_size_->setSingleStep(0.1);
    spin_robot_marker_size_->setValue(robot_marker_size_m_);
    spin_robot_marker_size_->setToolTip("Approximate base length of the robot heading triangle");
    size_layout->addWidget(spin_robot_marker_size_);
    layout->addLayout(size_layout);
    
    
    chk_show_robot_ = new QCheckBox("Show live robot overlay");
    chk_show_robot_->setChecked(true);
    
    btn_clear_robot_trail_ = new QPushButton("Clear trail");
    btn_clear_robot_trail_->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
    
    lbl_robot_status_ = new QLabel();
    lbl_robot_status_->setStyleSheet("color: #a66f00; font-size: 10px;");
    updateRobotStatusLabel(false);
    
    connect(chk_show_robot_, &QCheckBox::toggled, this, [this]() {
        refreshPlot();
    });
    connect(btn_clear_robot_trail_, &QPushButton::clicked, this, &CoverageGUI::clearRobotTrail);
    connect(txt_robot_topic_, &QLineEdit::editingFinished, this, [this]() {
        QString trimmed = txt_robot_topic_->text().trimmed();
        if (trimmed.isEmpty()) {
            trimmed = "/Odometry_tilt_corrected_diff";
            txt_robot_topic_->setText(trimmed);
        }
        if (trimmed == robot_odom_topic_) {
            return;
        }
        robot_odom_topic_ = trimmed;
        QSettings settings("PilotControl", "F2CCoveragePlanner");
        settings.setValue("robot_odom_topic", robot_odom_topic_);
        updateRobotStatusLabel(false);
        setupRobotTrackingSubscription();
    });
    connect(spin_robot_marker_size_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        robot_marker_size_m_ = value;
        QSettings settings("PilotControl", "F2CCoveragePlanner");
        settings.setValue("robot_marker_size_m", robot_marker_size_m_);
        plot_->setRobotMarkerSize(robot_marker_size_m_);
        refreshPlot();
    });
    
    layout->addWidget(chk_show_robot_);
    layout->addWidget(btn_clear_robot_trail_);
    layout->addWidget(lbl_robot_status_);
    
    return box;
}

QGroupBox* CoverageGUI::buildHeightControls() {
    QGroupBox* box = new QGroupBox("Height Cropping & 3D View");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    // Info label explaining the Z range filtering
    QLabel* info_label = new QLabel("Filter points relative to robot origin (Z=0):");
    info_label->setStyleSheet("color: #666; font-size: 10px;");
    v->addWidget(info_label);
    
    // Z minimum control (can be negative for below robot)
    QHBoxLayout* h_min = new QHBoxLayout();
    h_min->addWidget(new QLabel("Z min (m):"));
    spin_z_min_ = new QDoubleSpinBox();
    spin_z_min_->setRange(-50.0, 50.0);
    spin_z_min_->setSingleStep(0.05);
    spin_z_min_->setValue(-0.1);  // Default: 0.1m below robot
    spin_z_min_->setToolTip("Minimum Z value (negative = below robot origin)");
    h_min->addWidget(spin_z_min_);
    v->addLayout(h_min);
    
    // Z maximum control
    QHBoxLayout* h_max = new QHBoxLayout();
    h_max->addWidget(new QLabel("Z max (m):"));
    spin_z_max_ = new QDoubleSpinBox();
    spin_z_max_->setRange(-50.0, 50.0);
    spin_z_max_->setSingleStep(0.05);
    spin_z_max_->setValue(0.1);  // Default: 0.1m above robot
    spin_z_max_->setToolTip("Maximum Z value (positive = above robot origin)");
    h_max->addWidget(spin_z_max_);
    v->addLayout(h_max);
    
    QPushButton* btn_apply = new QPushButton("Apply Height Crop");
    btn_apply->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
    btn_apply->setToolTip("Keep only points with Z between Z min and Z max\n"
                          "(relative to robot origin at Z=0)");
    connect(btn_apply, &QPushButton::clicked, this, &CoverageGUI::applyHeightCrop);
    v->addWidget(btn_apply);
    
    // 3D Visualization button
    QPushButton* btn_view3d = new QPushButton("View Point Cloud (3D)");
    btn_view3d->setIcon(style()->standardIcon(QStyle::SP_ComputerIcon));
    btn_view3d->setToolTip("Open interactive 3D viewer for the point cloud.\n"
                          "Controls:\n"
                          "  • Left mouse: Rotate view\n"
                          "  • Middle mouse / Shift+Left: Pan\n"
                          "  • Scroll wheel: Zoom\n"
                          "  • R: Reset camera\n"
                          "  • C: Show camera parameters\n"
                          "  • +/-: Increase/decrease point size\n"
                          "  • G: Toggle coordinate system\n"
                          "  • Q: Close viewer");
    connect(btn_view3d, &QPushButton::clicked, this, &CoverageGUI::showPointCloud3D);
    v->addWidget(btn_view3d);
    
    return box;
}

QGroupBox* CoverageGUI::buildDownsampleControls() {
    QGroupBox* box = new QGroupBox("Downsampling");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QHBoxLayout* method_row = new QHBoxLayout();
    method_row->addWidget(new QLabel("Method"));
    combo_downsample_ = new QComboBox();
    combo_downsample_->addItems({"None", "Random", "Voxel", "Statistical"});
    connect(combo_downsample_, &QComboBox::currentTextChanged, this, &CoverageGUI::updateDownsampleUI);
    method_row->addWidget(combo_downsample_);
    v->addLayout(method_row);
    
    // Random params
    group_random_ = new QGroupBox("Random settings");
    QHBoxLayout* rand_layout = new QHBoxLayout(group_random_);
    rand_layout->addWidget(new QLabel("Max points"));
    spin_max_points_ = new QSpinBox();
    spin_max_points_->setRange(100, 2000000);
    spin_max_points_->setValue(50000);
    rand_layout->addWidget(spin_max_points_);
    v->addWidget(group_random_);
    
    // Voxel params
    group_voxel_ = new QGroupBox("Voxel settings");
    QHBoxLayout* voxel_layout = new QHBoxLayout(group_voxel_);
    voxel_layout->addWidget(new QLabel("Voxel size"));
    spin_voxel_ = new QDoubleSpinBox();
    spin_voxel_->setRange(0.001, 1.0);
    spin_voxel_->setSingleStep(0.005);
    spin_voxel_->setValue(0.05);
    voxel_layout->addWidget(spin_voxel_);
    v->addWidget(group_voxel_);
    
    // Statistical params
    group_stat_ = new QGroupBox("Statistical settings");
    QHBoxLayout* stat_layout = new QHBoxLayout(group_stat_);
    stat_layout->addWidget(new QLabel("Mean K"));
    spin_mean_k_ = new QSpinBox();
    spin_mean_k_->setRange(5, 1000);
    spin_mean_k_->setValue(20);
    stat_layout->addWidget(spin_mean_k_);
    stat_layout->addWidget(new QLabel("Std Ratio"));
    spin_std_ratio_ = new QDoubleSpinBox();
    spin_std_ratio_->setRange(0.1, 5.0);
    spin_std_ratio_->setSingleStep(0.1);
    spin_std_ratio_->setValue(1.0);
    stat_layout->addWidget(spin_std_ratio_);
    v->addWidget(group_stat_);
    
    updateDownsampleUI("None");
    
    QPushButton* btn_down = new QPushButton("Downsample");
    btn_down->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
    connect(btn_down, &QPushButton::clicked, this, &CoverageGUI::applyDownsample);
    v->addWidget(btn_down);
    
    return box;
}

QGroupBox* CoverageGUI::buildHullControls() {
    QGroupBox* box = new QGroupBox("2D Projection & Concave Hull");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QHBoxLayout* h_method = new QHBoxLayout();
    h_method->addWidget(new QLabel("Method:"));
    combo_hull_method_ = new QComboBox();
    combo_hull_method_->addItem("AlphaShape", "alphashape");
    combo_hull_method_->addItem("Delaunay", "delaunay");
    combo_hull_method_->addItem("Grid", "grid");
    h_method->addWidget(combo_hull_method_);
    v->addLayout(h_method);
    
    QHBoxLayout* h = new QHBoxLayout();
    h->addWidget(new QLabel("Parameter:"));
    spin_alpha_ = new QDoubleSpinBox();
    spin_alpha_->setRange(0.01, 10.0);
    spin_alpha_->setSingleStep(0.1);
    spin_alpha_->setValue(1.5);
    spin_alpha_->setToolTip("AlphaShape: smaller=more detail. Grid: grid cell size");
    h->addWidget(spin_alpha_);
    v->addLayout(h);
    
    QPushButton* btn_proj = new QPushButton("Project to 2D & Compute Hull");
    btn_proj->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
    connect(btn_proj, &QPushButton::clicked, this, &CoverageGUI::computeHull);
    v->addWidget(btn_proj);
    
    return box;
}

QGroupBox* CoverageGUI::buildSimplifyControls() {
    QGroupBox* box = new QGroupBox("Polygon Simplification");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QHBoxLayout* h = new QHBoxLayout();
    h->addWidget(new QLabel("Tolerance"));
    spin_simplify_ = new QDoubleSpinBox();
    spin_simplify_->setRange(0.0, 5.0);
    spin_simplify_->setSingleStep(0.05);
    spin_simplify_->setValue(0.1);
    h->addWidget(spin_simplify_);
    v->addLayout(h);
    
    QPushButton* btn_simplify = new QPushButton("Simplify Polygon");
    btn_simplify->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
    connect(btn_simplify, &QPushButton::clicked, this, &CoverageGUI::simplifyPolygon);
    v->addWidget(btn_simplify);
    
    return box;
}

QGroupBox* CoverageGUI::buildPathPlanningControls() {
    QGroupBox* box = new QGroupBox("Path Planning");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    // Mode selector
    QHBoxLayout* mode_layout = new QHBoxLayout();
    mode_layout->addWidget(new QLabel("Mode:"));
    radio_mode_f2c_ = new QRadioButton("F2C Coverage");
    radio_mode_custom_ = new QRadioButton("Custom Path");
    radio_mode_f2c_->setChecked(true);
    radio_mode_f2c_->setToolTip("Use Fields2Cover library for coverage path planning");
    radio_mode_custom_->setToolTip("Draw custom waypoints on the map");
    mode_layout->addWidget(radio_mode_f2c_);
    mode_layout->addWidget(radio_mode_custom_);
    mode_layout->addStretch();
    v->addLayout(mode_layout);
    
    // F2C controls container
    f2c_controls_widget_ = buildF2CControls();
    v->addWidget(f2c_controls_widget_);
    
    // Custom path controls container
    custom_controls_widget_ = buildCustomPathControls();
    custom_controls_widget_->setVisible(false);
    v->addWidget(custom_controls_widget_);
    
    return box;
}

QWidget* CoverageGUI::buildF2CControls() {
    QWidget* widget = new QWidget();
    QVBoxLayout* v = new QVBoxLayout(widget);
    v->setContentsMargins(0, 0, 0, 0);
    
    // Swath width
    QHBoxLayout* h1 = new QHBoxLayout();
    h1->addWidget(new QLabel("Swath width"));
    spin_swath_ = new QDoubleSpinBox();
    spin_swath_->setRange(0.05, 10.0);
    spin_swath_->setSingleStep(0.05);
    spin_swath_->setValue(1.0);
    h1->addWidget(spin_swath_);
    v->addLayout(h1);
    
    // Headland width
    QHBoxLayout* h2 = new QHBoxLayout();
    h2->addWidget(new QLabel("Headland width"));
    spin_headland_ = new QDoubleSpinBox();
    spin_headland_->setRange(0.0, 10.0);
    spin_headland_->setSingleStep(0.1);
    spin_headland_->setValue(1.0);
    h2->addWidget(spin_headland_);
    v->addLayout(h2);
    
    // Turn radius
    QHBoxLayout* h3 = new QHBoxLayout();
    h3->addWidget(new QLabel("Turn radius"));
    spin_turn_ = new QDoubleSpinBox();
    spin_turn_->setRange(0.0, 20.0);
    spin_turn_->setSingleStep(0.1);
    spin_turn_->setValue(0.5);
    h3->addWidget(spin_turn_);
    v->addLayout(h3);
    
    // Auto-align
    chk_auto_align_ = new QCheckBox("Auto-align to building");
    v->addWidget(chk_auto_align_);
    
    QHBoxLayout* align_box = new QHBoxLayout();
    radio_long_ = new QRadioButton("Parallel (long edge)");
    radio_perp_ = new QRadioButton("Perpendicular");
    radio_perp_->setChecked(true);
    align_box->addWidget(radio_long_);
    align_box->addWidget(radio_perp_);
    v->addLayout(align_box);
    
    // Route pattern
    QHBoxLayout* route_layout = new QHBoxLayout();
    route_layout->addWidget(new QLabel("Route pattern"));
    combo_route_pattern_ = new QComboBox();
    combo_route_pattern_->addItem("Boustrophedon", "boustro");
    combo_route_pattern_->addItem("Snake", "snake");
    combo_route_pattern_->addItem("Spiral", "spiral");
    route_layout->addWidget(combo_route_pattern_);
    v->addLayout(route_layout);
    
    // Path planner
    QHBoxLayout* planner_layout = new QHBoxLayout();
    planner_layout->addWidget(new QLabel("Path planner"));
    combo_path_planner_ = new QComboBox();
    combo_path_planner_->addItem("Dubins curves", "dubins");
    combo_path_planner_->addItem("Dubins curves (CC)", "dubins_cc");
    combo_path_planner_->addItem("Reeds-Shepp", "reeds");
    combo_path_planner_->addItem("Reeds-Shepp (HC)", "reeds_hc");
    combo_path_planner_->addItem("Straight", "none");
    planner_layout->addWidget(combo_path_planner_);
    v->addLayout(planner_layout);
    
    // Axial turns
    chk_axial_turns_ = new QCheckBox("Use axial turns (zero radius)");
    chk_axial_turns_->setToolTip("For robots that can turn in place");
    v->addWidget(chk_axial_turns_);
    
    // Decomposition
    chk_decomposition_ = new QCheckBox("Use decomposition (concave fields)");
    v->addWidget(chk_decomposition_);
    
    QHBoxLayout* decomp_layout = new QHBoxLayout();
    decomp_layout->addWidget(new QLabel("Decomposition type"));
    combo_decomp_type_ = new QComboBox();
    combo_decomp_type_->addItem("Boustrophedon", "boustrophedon");
    combo_decomp_type_->addItem("Trapezoidal", "trapezoidal");
    decomp_layout->addWidget(combo_decomp_type_);
    v->addLayout(decomp_layout);
    
    // ROI controls
    QHBoxLayout* roi_box = new QHBoxLayout();
    btn_roi_ = new QPushButton("Select ROI");
    btn_roi_->setCheckable(true);
    btn_roi_->setIcon(style()->standardIcon(QStyle::SP_DialogYesButton));
    connect(btn_roi_, &QPushButton::clicked, this, &CoverageGUI::toggleROISelection);
    roi_box->addWidget(btn_roi_);
    
    btn_roi_clear_ = new QPushButton("Clear ROI");
    btn_roi_clear_->setIcon(style()->standardIcon(QStyle::SP_DialogCancelButton));
    connect(btn_roi_clear_, &QPushButton::clicked, this, &CoverageGUI::clearROI);
    roi_box->addWidget(btn_roi_clear_);
    v->addLayout(roi_box);
    
    QHBoxLayout* roi_actions = new QHBoxLayout();
    btn_roi_finish_ = new QPushButton("Finish");
    btn_roi_finish_->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
    connect(btn_roi_finish_, &QPushButton::clicked, this, &CoverageGUI::finishSelection);
    roi_actions->addWidget(btn_roi_finish_);
    
    btn_roi_undo_ = new QPushButton("Undo Point");
    btn_roi_undo_->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
    connect(btn_roi_undo_, &QPushButton::clicked, this, &CoverageGUI::undoSelectionPoint);
    roi_actions->addWidget(btn_roi_undo_);
    v->addLayout(roi_actions);
    
    lbl_roi_ = new QLabel("ROI: none");
    v->addWidget(lbl_roi_);
    
    // Obstacle controls
    QHBoxLayout* obstacle_box = new QHBoxLayout();
    btn_obstacle_ = new QPushButton("Add Obstacle");
    btn_obstacle_->setCheckable(true);
    btn_obstacle_->setIcon(style()->standardIcon(QStyle::SP_MessageBoxWarning));
    connect(btn_obstacle_, &QPushButton::clicked, this, &CoverageGUI::toggleObstacleSelection);
    obstacle_box->addWidget(btn_obstacle_);
    
    btn_obstacle_clear_ = new QPushButton("Clear Obstacles");
    btn_obstacle_clear_->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
    connect(btn_obstacle_clear_, &QPushButton::clicked, this, &CoverageGUI::clearObstacles);
    obstacle_box->addWidget(btn_obstacle_clear_);
    v->addLayout(obstacle_box);
    
    lbl_obstacles_ = new QLabel("Obstacles: 0");
    v->addWidget(lbl_obstacles_);
    
    // Generation buttons
    QPushButton* btn_swaths = new QPushButton("Generate Swaths");
    btn_swaths->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
    connect(btn_swaths, &QPushButton::clicked, this, &CoverageGUI::generateSwaths);
    v->addWidget(btn_swaths);
    
    QPushButton* btn_route = new QPushButton("Generate Route");
    btn_route->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
    connect(btn_route, &QPushButton::clicked, this, &CoverageGUI::generateRoute);
    v->addWidget(btn_route);
    
    QPushButton* btn_clear = new QPushButton("Clear Swaths/Path");
    btn_clear->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
    connect(btn_clear, &QPushButton::clicked, this, &CoverageGUI::clearCoverage);
    v->addWidget(btn_clear);
    
    QPushButton* btn_path = new QPushButton("Generate Path");
    btn_path->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    connect(btn_path, &QPushButton::clicked, this, &CoverageGUI::generatePath);
    v->addWidget(btn_path);
    
    return widget;
}

QWidget* CoverageGUI::buildCustomPathControls() {
    QWidget* widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(0, 0, 0, 0);
    
    QLabel* instructions = new QLabel(
        "Click 'Enable drawing' then click on the map to drop waypoints.\n"
        "Use 'Publish Waypoints' below to send to robot.");
    instructions->setWordWrap(true);
    instructions->setStyleSheet("color: #666; font-size: 10px;");
    layout->addWidget(instructions);
    
    btn_custom_draw_ = new QPushButton("Enable drawing");
    btn_custom_draw_->setCheckable(true);
    btn_custom_draw_->setStyleSheet("QPushButton:checked { background-color: #81C784; }");
    layout->addWidget(btn_custom_draw_);
    
    QHBoxLayout* edit_layout = new QHBoxLayout();
    btn_custom_undo_ = new QPushButton("Undo Last");
    btn_custom_undo_->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
    btn_custom_clear_ = new QPushButton("Clear All");
    btn_custom_clear_->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
    edit_layout->addWidget(btn_custom_undo_);
    edit_layout->addWidget(btn_custom_clear_);
    layout->addLayout(edit_layout);
    
    list_custom_points_ = new QListWidget();
    list_custom_points_->setMaximumHeight(150);
    layout->addWidget(list_custom_points_);
    
    lbl_custom_status_ = new QLabel("No custom waypoints yet.");
    lbl_custom_status_->setStyleSheet("color: #777; font-size: 10px;");
    layout->addWidget(lbl_custom_status_);
    
    return widget;
}

QGroupBox* CoverageGUI::buildExportControls() {
    QGroupBox* box = new QGroupBox("Export & Navigation");
    QVBoxLayout* v = new QVBoxLayout(box);

    QPushButton* btn_export_path = new QPushButton("Export Path CSV");
    btn_export_path->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
    connect(btn_export_path, &QPushButton::clicked, this, &CoverageGUI::exportPathCSV);
    v->addWidget(btn_export_path);

    // Add waypoint publishing buttons (work for both F2C and Custom modes)
    btn_publish_waypoints_ = new QPushButton("📡 Publish Waypoints to Robot");
    btn_publish_waypoints_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
    connect(btn_publish_waypoints_, &QPushButton::clicked, this, &CoverageGUI::publishWaypoints);
    v->addWidget(btn_publish_waypoints_);

    btn_start_navigation_ = new QPushButton("▶️ Start Navigation");
    btn_start_navigation_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
    btn_start_navigation_->setEnabled(false);  // Initially disabled
    connect(btn_start_navigation_, &QPushButton::clicked, this, &CoverageGUI::startNavigation);
    v->addWidget(btn_start_navigation_);

    // Reprojection error analysis section
    QFrame* sep = new QFrame();
    sep->setFrameShape(QFrame::HLine);
    sep->setFrameShadow(QFrame::Sunken);
    v->addWidget(sep);
    
    QLabel* reproj_title = new QLabel("Path Accuracy Analysis");
    reproj_title->setStyleSheet("font-weight: bold; margin-top: 5px;");
    v->addWidget(reproj_title);
    
    btn_compute_reproj_ = new QPushButton("📊 Compute Reprojection Error");
    btn_compute_reproj_->setToolTip("Compare robot trail to planned path (within 1m)");
    connect(btn_compute_reproj_, &QPushButton::clicked, this, &CoverageGUI::computeReprojectionError);
    v->addWidget(btn_compute_reproj_);
    
    btn_clear_reproj_ = new QPushButton("Clear Reprojection");
    btn_clear_reproj_->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
    connect(btn_clear_reproj_, &QPushButton::clicked, this, &CoverageGUI::clearReprojectionError);
    v->addWidget(btn_clear_reproj_);
    
    lbl_reproj_status_ = new QLabel("No reprojection computed");
    lbl_reproj_status_->setStyleSheet("color: #666; font-size: 10px;");
    v->addWidget(lbl_reproj_status_);

    return box;
}

void CoverageGUI::setStatus(const QString& text, int timeout_ms) {
    status_bar_->showMessage(text, timeout_ms);
}

void CoverageGUI::showProgress(bool show, const QString& text) {
    progress_bar_->setVisible(show);
    if (show && !text.isEmpty()) {
        setStatus(text);
    }
}

void CoverageGUI::updateProgress(int percent, const QString& text) {
    progress_bar_->setValue(percent);
    if (!text.isEmpty()) {
        setStatus(text);
    }
}

CoverageConfig CoverageGUI::currentConfig() const {
    CoverageConfig cfg;
    cfg.swath_width = spin_swath_->value();
    cfg.headland_width = spin_headland_->value();
    cfg.turn_radius = spin_turn_->value();
    cfg.auto_align = chk_auto_align_->isChecked();
    cfg.align_mode = radio_perp_->isChecked() ? "perp" : "long";
    cfg.route_pattern = combo_route_pattern_->currentData().toString().toStdString();
    cfg.path_planner = combo_path_planner_->currentData().toString().toStdString();
    cfg.use_decomposition = chk_decomposition_->isChecked();
    cfg.decomposition_type = combo_decomp_type_->currentData().toString().toStdString();
    cfg.use_axial_turns = chk_axial_turns_->isChecked();
    return cfg;
}

void CoverageGUI::refreshPlot() {
    plot_->setPoints(xy_2d_);
    plot_->setPolygon(polygon_);
    plot_->setROI(roi_polygon_);
    plot_->setObstacles(obstacles_);
    plot_->setSwaths(swaths_);
    
    // Convert route PathStateList to use
    plot_->setRoute(route_);
    plot_->setPath(path_);
    plot_->setCustomPath(custom_waypoints_, custom_waypoints_visited_);
    plot_->setShowCustomPath(isCustomModeActive() && !custom_waypoints_.empty());
    
    std::optional<PathState> pose_copy;
    std::vector<Point2D> trail_copy;
    std::chrono::steady_clock::time_point last_update_copy{};
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        pose_copy = robot_pose_state_;
        trail_copy = robot_trail_;
        last_update_copy = last_robot_update_;
    }
    
    bool show_robot = !chk_show_robot_ || chk_show_robot_->isChecked();
    if (!show_robot) {
        pose_copy.reset();
        trail_copy.clear();
    }
    
    plot_->setRobotPose(pose_copy);
    plot_->setRobotTrail(trail_copy);
    
    if (fit_view_pending_) {
        plot_->resetView();
        fit_view_pending_ = false;
    } else {
        plot_->update();
    }
    
    bool pose_fresh = false;
    if (pose_copy.has_value() && last_update_copy.time_since_epoch().count() > 0) {
        auto age = std::chrono::steady_clock::now() - last_update_copy;
        pose_fresh = age < std::chrono::seconds(2);
    }
    updateRobotStatusLabel(pose_fresh);
}

Polygon2D CoverageGUI::effectivePolygon() const {
    if (!roi_polygon_.empty()) {
        return roi_polygon_;  // TODO: Implement proper intersection
    }
    return polygon_;
}

void CoverageGUI::updateDownsampleUI(const QString& method) {
    QString m = method.toLower();
    group_random_->setVisible(m == "random");
    group_voxel_->setVisible(m == "voxel");
    group_stat_->setVisible(m == "statistical");
}

// Slot implementations

void CoverageGUI::loadPointCloud() {
    QString path = QFileDialog::getOpenFileName(
        this, "Select point cloud", "", "Point Clouds (*.pcd *.ply *.xyz)");
    
    if (path.isEmpty()) return;
    
    showProgress(true, "Loading point cloud...");
    
    try {
        pcd_points_ = loadPointCloudFile(path.toStdString());
        filtered_points_ = pcd_points_;
        
        loaded_file_ = path;
        lbl_file_->setText(QFileInfo(path).fileName());
        
        // Clear old data
        polygon_.clear();
        roi_polygon_.clear();
        obstacles_.clear();
        swaths_.clear();
        route_.clear();
        path_.clear();
        
        lbl_roi_->setText("ROI: none");
        lbl_obstacles_->setText("Obstacles: 0");
        
        plot_->clearAll();
        
        // Project points to 2D immediately for visualization
        xy_2d_.clear();
        xy_2d_.reserve(pcd_points_->size());
        for (const auto& pt : pcd_points_->points) {
            xy_2d_.emplace_back(pt.x, pt.y);
        }
        
        scheduleFitToView();
        refreshPlot();
        setStatus(QString("Loaded %1 points").arg(pcd_points_->size()), 4000);
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load: %1").arg(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::clearRobotTrail() {
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        robot_trail_.clear();
    }
    refreshPlot();
}

void CoverageGUI::fetchLatestMapFromRobot() {
    showProgress(true, QString("Connecting to %1...").arg(robot_host_));
    
    // Build SSH command to find the latest .pcd file on robot
    QString find_cmd = QString(
        "ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no %1@%2 "
        "\"find %3 -name '*.pcd' -type f -printf '%T@ %p\\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-\""
    ).arg(robot_user_, robot_host_, robot_data_path_);
    
    QProcess find_process;
    find_process.start("bash", QStringList() << "-c" << find_cmd);
    
    if (!find_process.waitForFinished(10000)) {
        showProgress(false);
        QMessageBox::warning(this, "Connection Failed", 
            QString("Could not connect to robot.\nCheck if:\n"
                    "• Robot is powered on\n"
                    "• Microhard is connected (%1)\n"
                    "• SSH keys are configured for %2@%1")
                .arg(robot_host_, robot_user_));
        return;
    }
    
    if (find_process.exitCode() != 0) {
        showProgress(false);
        QString error = QString::fromUtf8(find_process.readAllStandardError());
        QMessageBox::warning(this, "SSH Error", "SSH command failed:\n" + error);
        return;
    }
    
    QString remote_path = QString::fromUtf8(find_process.readAllStandardOutput()).trimmed();
    
    if (remote_path.isEmpty()) {
        showProgress(false);
        QMessageBox::warning(this, "No Maps Found", 
            "No .pcd map files found on robot at " + robot_data_path_);
        return;
    }
    
    // Create local folder structure: ~/Roofus_maps/December_01_2025/
    QDate today = QDate::currentDate();
    QString day_folder = today.toString("MMMM_dd_yyyy");  // e.g., "December_01_2025"
    QString local_dir = local_map_base_ + "/" + day_folder;
    QDir().mkpath(local_dir);
    
    // Extract filename and create local path
    QFileInfo remote_info(remote_path);
    QString local_path = local_dir + "/" + remote_info.fileName();
    
    // Check if file already exists locally
    if (QFile::exists(local_path)) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, "File Exists", 
            QString("Map already exists locally:\n%1\n\nOverwrite?").arg(local_path),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply != QMessageBox::Yes) {
            showProgress(false);
            // Offer to load existing file
            reply = QMessageBox::question(this, "Load Existing?", 
                "Load the existing local map instead?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            if (reply == QMessageBox::Yes) {
                loadPointCloudFromPath(local_path);
            }
            return;
        }
    }
    
    showProgress(true, "Downloading: " + remote_info.fileName());
    
    // SCP the file to local machine
    QString scp_cmd = QString(
        "scp -o ConnectTimeout=10 -o StrictHostKeyChecking=no %1@%2:\"%3\" \"%4\""
    ).arg(robot_user_, robot_host_, remote_path, local_path);
    
    QProcess scp_process;
    scp_process.start("bash", QStringList() << "-c" << scp_cmd);
    
    if (!scp_process.waitForFinished(180000)) {  // 3 min timeout for large files
        showProgress(false);
        QMessageBox::warning(this, "Download Failed", 
            "File transfer timed out.\nThe map file may be too large or connection is slow.");
        return;
    }
    
    if (scp_process.exitCode() != 0) {
        showProgress(false);
        QString error = QString::fromUtf8(scp_process.readAllStandardError());
        QMessageBox::warning(this, "Download Failed", 
            "Could not download map file:\n" + error);
        return;
    }
    
    showProgress(false);
    
    // Verify file exists locally
    QFileInfo local_info(local_path);
    if (!local_info.exists() || local_info.size() == 0) {
        QMessageBox::warning(this, "Download Failed", "Map file was not saved correctly.");
        return;
    }
    
    // Show success and offer to load
    QString msg = QString(
        "✅ Map downloaded successfully!\n\n"
        "File: %1\n"
        "Size: %2 MB\n"
        "Saved to: %3\n\n"
        "Load this map now?"
    ).arg(local_info.fileName())
     .arg(local_info.size() / (1024.0 * 1024.0), 0, 'f', 2)
     .arg(local_dir);
    
    QMessageBox::StandardButton reply = QMessageBox::question(
        this, "Download Complete", msg,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    
    if (reply == QMessageBox::Yes) {
        loadPointCloudFromPath(local_path);
    }
    
    setStatus("Map saved: " + local_path);
}

void CoverageGUI::loadPointCloudFromPath(const QString& path) {
    showProgress(true, "Loading point cloud...");
    
    try {
        pcd_points_ = loadPointCloudFile(path.toStdString());
        filtered_points_ = pcd_points_;
        
        loaded_file_ = path;
        lbl_file_->setText(QFileInfo(path).fileName());
        
        // Clear old data
        polygon_.clear();
        roi_polygon_.clear();
        obstacles_.clear();
        swaths_.clear();
        route_.clear();
        path_.clear();
        
        lbl_roi_->setText("ROI: none");
        lbl_obstacles_->setText("Obstacles: 0");
        
        plot_->clearAll();
        
        // Project points to 2D immediately for visualization
        xy_2d_.clear();
        xy_2d_.reserve(pcd_points_->size());
        for (const auto& pt : pcd_points_->points) {
            xy_2d_.emplace_back(pt.x, pt.y);
        }
        
        scheduleFitToView();
        refreshPlot();
        setStatus(QString("Loaded %1 points from %2").arg(pcd_points_->size()).arg(QFileInfo(path).fileName()), 4000);
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load: %1").arg(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::applyHeightCrop() {
    if (!pcd_points_ || pcd_points_->empty()) {
        QMessageBox::warning(this, "Warning", "Load a point cloud first.");
        return;
    }
    
    double z_min = spin_z_min_->value();
    double z_max = spin_z_max_->value();
    
    showProgress(true, QString("Applying height crop [%1, %2]m...").arg(z_min).arg(z_max));
    
    try {
        // Use the new Z range filter (relative to robot origin Z=0)
        filtered_points_ = filterByZRange(pcd_points_, z_min, z_max);
        
        // Project to 2D for visualization
        xy_2d_.clear();
        xy_2d_.reserve(filtered_points_->size());
        for (const auto& pt : filtered_points_->points) {
            xy_2d_.emplace_back(pt.x, pt.y);
        }
        
        // Clear previous polygon/coverage data
        polygon_.clear();
        swaths_.clear();
        route_.clear();
        path_.clear();
        
        scheduleFitToView();
        refreshPlot();
        setStatus(QString("Filtered to %1 points (Z: %2 to %3 m)")
                  .arg(filtered_points_->size())
                  .arg(z_min, 0, 'f', 2)
                  .arg(z_max, 0, 'f', 2), 4000);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::showPointCloud3D() {
    // Use filtered cloud if available, otherwise use raw cloud
    PointCloudPtr cloud = filtered_points_ ? filtered_points_ : pcd_points_;
    
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Warning", 
            "Load a point cloud first.\n\n"
            "Use 'Load PCD / PLY / XYZ' to load a file.");
        return;
    }
    
    setStatus(QString("Preparing 3D viewer with %1 points...").arg(cloud->size()));
    showProgress(true, "Saving temporary point cloud...");
    
    // Save point cloud to a temporary file (avoids VTK/Qt threading conflicts)
    std::string temp_pcd = "/tmp/f2c_viewer_temp.pcd";
    std::string temp_path_csv = "/tmp/f2c_viewer_path.csv";
    
    // Create colored point cloud based on Z height
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(cloud->size());
    
    // Calculate bounds
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& pt : cloud->points) {
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }
    
    float z_range = max_z - min_z;
    if (z_range < 0.001f) z_range = 1.0f;
    
    // Apply height-based coloring
    for (const auto& pt : cloud->points) {
        pcl::PointXYZRGB colored_pt;
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        
        float t = (pt.z - min_z) / z_range;
        t = std::max(0.0f, std::min(1.0f, t));
        
        // Rainbow gradient: blue -> cyan -> green -> yellow -> red
        uint8_t r, g, b;
        if (t < 0.25f) {
            float s = t / 0.25f;
            r = 0; g = static_cast<uint8_t>(255 * s); b = 255;
        } else if (t < 0.5f) {
            float s = (t - 0.25f) / 0.25f;
            r = 0; g = 255; b = static_cast<uint8_t>(255 * (1 - s));
        } else if (t < 0.75f) {
            float s = (t - 0.5f) / 0.25f;
            r = static_cast<uint8_t>(255 * s); g = 255; b = 0;
        } else {
            float s = (t - 0.75f) / 0.25f;
            r = 255; g = static_cast<uint8_t>(255 * (1 - s)); b = 0;
        }
        
        colored_pt.r = r;
        colored_pt.g = g;
        colored_pt.b = b;
        colored_cloud->push_back(colored_pt);
    }
    
    // Save to temp file
    if (pcl::io::savePCDFileBinary(temp_pcd, *colored_cloud) != 0) {
        QMessageBox::critical(this, "Error", "Failed to save temporary point cloud file.");
        showProgress(false);
        return;
    }
    
    // Save path to CSV if available
    bool has_path = !path_.empty();
    if (has_path) {
        std::ofstream path_out(temp_path_csv);
        path_out << "x,y,z\n";
        for (const auto& state : path_) {
            // Use Z=0 for path points (robot operates on XY plane at origin height)
            path_out << std::fixed << std::setprecision(6) 
                     << state.point.x << "," << state.point.y << ",0.0\n";
        }
        path_out.close();
    }
    
    showProgress(false);
    
    // Try different viewer options in order of preference
    std::string viewer_cmd;
    
    // Check for Open3D via Python (preferred - supports path visualization)
    if (system("python3 -c 'import open3d' > /dev/null 2>&1") == 0) {
        // Create a Python script for Open3D visualization with origin and path
        std::string py_script = R"PYTHON(
import open3d as o3d
import numpy as np
import sys
import os

pcd_file = sys.argv[1]
path_file = sys.argv[2] if len(sys.argv) > 2 else None

pcd = o3d.io.read_point_cloud(pcd_file)
print(f"Loaded {len(pcd.points)} points")

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="F2C 3D Viewer - Point Cloud + Path", width=1200, height=900)
vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.point_size = 2.0
opt.background_color = np.array([0.08, 0.08, 0.12])
opt.show_coordinate_frame = True

origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
origin_sphere.translate([0, 0, 0])
origin_sphere.paint_uniform_color([0.9, 0.1, 0.1])
vis.add_geometry(origin_sphere)

axis_length = 1.0
axis_radius = 0.03

x_axis = o3d.geometry.TriangleMesh.create_cylinder(radius=axis_radius, height=axis_length)
x_axis.rotate(o3d.geometry.get_rotation_matrix_from_xyz([0, np.pi/2, 0]), center=[0,0,0])
x_axis.translate([axis_length/2, 0, 0])
x_axis.paint_uniform_color([1, 0.2, 0.2])
vis.add_geometry(x_axis)

y_axis = o3d.geometry.TriangleMesh.create_cylinder(radius=axis_radius, height=axis_length)
y_axis.rotate(o3d.geometry.get_rotation_matrix_from_xyz([-np.pi/2, 0, 0]), center=[0,0,0])
y_axis.translate([0, axis_length/2, 0])
y_axis.paint_uniform_color([0.2, 1, 0.2])
vis.add_geometry(y_axis)

z_axis = o3d.geometry.TriangleMesh.create_cylinder(radius=axis_radius, height=axis_length)
z_axis.translate([0, 0, axis_length/2])
z_axis.paint_uniform_color([0.2, 0.2, 1])
vis.add_geometry(z_axis)

if path_file and os.path.exists(path_file):
    try:
        path_data = np.loadtxt(path_file, delimiter=',', skiprows=1)
        if len(path_data) > 1:
            print(f"Loaded path with {len(path_data)} waypoints")
            
            lines = [[i, i+1] for i in range(len(path_data)-1)]
            colors = [[0, 0.9, 0.3] for _ in lines]
            
            path_line = o3d.geometry.LineSet()
            path_line.points = o3d.utility.Vector3dVector(path_data)
            path_line.lines = o3d.utility.Vector2iVector(lines)
            path_line.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry(path_line)
            
            start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            start_sphere.translate(path_data[0])
            start_sphere.paint_uniform_color([0, 1, 0])
            vis.add_geometry(start_sphere)
            
            end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            end_sphere.translate(path_data[-1])
            end_sphere.paint_uniform_color([1, 1, 0])
            vis.add_geometry(end_sphere)
            
            print("Path visualization added (green line, green=start, yellow=end)")
    except Exception as e:
        print(f"Could not load path: {e}")

ctr = vis.get_view_control()
ctr.set_zoom(0.7)

print("\nControls:")
print("  Left drag: Rotate | Scroll: Zoom | Shift+drag: Pan")
print("  R: Reset view | Q: Close")
print("\nLegend:")
print("  Red sphere: Origin (0,0,0)")
print("  RGB axes: X(red), Y(green), Z(blue)")
print("  Green line: Coverage path")

vis.run()
vis.destroy_window()
)PYTHON";
        
        std::string py_file = "/tmp/f2c_viewer.py";
        std::ofstream py_out(py_file);
        py_out << py_script;
        py_out.close();
        
        if (has_path) {
            viewer_cmd = "python3 " + py_file + " " + temp_pcd + " " + temp_path_csv + " &";
            setStatus("Opening Open3D viewer with path (close window when done)", 5000);
        } else {
            viewer_cmd = "python3 " + py_file + " " + temp_pcd + " &";
            setStatus("Opening Open3D viewer (close window when done)", 5000);
        }
    }
    // Check for pcl_viewer (fallback - no path support but shows coordinate frame)
    else if (system("which pcl_viewer > /dev/null 2>&1") == 0) {
        // pcl_viewer with coordinate axes
        viewer_cmd = "pcl_viewer " + temp_pcd + " -ps 2 -ax 1.0 &";
        if (has_path) {
            setStatus("Opening pcl_viewer (path not shown - install Open3D for path viz)", 5000);
        } else {
            setStatus("Opening pcl_viewer (press Q to close)", 5000);
        }
    }
    // Check for CloudCompare
    else if (system("which CloudCompare > /dev/null 2>&1") == 0) {
        viewer_cmd = "CloudCompare " + temp_pcd + " &";
        setStatus("Opening CloudCompare", 5000);
    }
    else {
        QMessageBox::warning(this, "No 3D Viewer Found",
            "Could not find a suitable 3D point cloud viewer.\n\n"
            "Please install one of the following:\n"
            "  • Open3D (pip install open3d) - recommended for path visualization\n"
            "  • pcl-tools (sudo apt install pcl-tools)\n"
            "  • CloudCompare (sudo apt install cloudcompare)\n\n"
            "The colored point cloud has been saved to:\n" + 
            QString::fromStdString(temp_pcd));
        return;
    }
    
    // Launch viewer in background
    int result = system(viewer_cmd.c_str());
    if (result != 0) {
        QMessageBox::warning(this, "Warning", 
            "Failed to launch 3D viewer.\n"
            "You can manually open the file:\n" + QString::fromStdString(temp_pcd));
    }
}

void CoverageGUI::applyDownsample() {
    if (!filtered_points_ || filtered_points_->empty()) {
        QMessageBox::warning(this, "Warning", "Apply height crop first.");
        return;
    }
    
    QString method = combo_downsample_->currentText().toLower();
    if (method == "none") {
        setStatus("Downsampling skipped", 3000);
        return;
    }
    
    showProgress(true, QString("Applying %1 downsampling...").arg(method));
    
    try {
        if (method == "random") {
            filtered_points_ = subsampleRandom(filtered_points_, spin_max_points_->value());
        } else if (method == "voxel") {
            filtered_points_ = downsampleVoxel(filtered_points_, spin_voxel_->value());
        } else if (method == "statistical") {
            filtered_points_ = downsampleStatistical(filtered_points_, 
                                                     spin_mean_k_->value(),
                                                     spin_std_ratio_->value());
        }
        setStatus(QString("Downsampled to %1 points").arg(filtered_points_->size()), 4000);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::computeHull() {
    if (!filtered_points_ || filtered_points_->empty()) {
        QMessageBox::warning(this, "Warning", "Apply filtering first.");
        return;
    }
    
    showProgress(true, "Computing hull...");
    
    try {
        // Project to 2D
        xy_2d_.clear();
        xy_2d_.reserve(filtered_points_->size());
        for (const auto& pt : filtered_points_->points) {
            xy_2d_.emplace_back(pt.x, pt.y);
        }
        
        QString method = combo_hull_method_->currentData().toString();
        polygon_ = computeConcaveHull(xy_2d_, spin_alpha_->value(), method.toStdString());
        
        scheduleFitToView();
        refreshPlot();
        setStatus(QString("Hull computed with %1 vertices").arg(polygon_.size()), 4000);
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::simplifyPolygon() {
    if (polygon_.empty()) {
        QMessageBox::warning(this, "Warning", "Compute hull first.");
        return;
    }
    
    showProgress(true, "Simplifying polygon...");
    
    try {
        polygon_ = f2c_cpp::simplifyPolygon(polygon_, spin_simplify_->value());
        scheduleFitToView();
        refreshPlot();
        setStatus(QString("Simplified to %1 vertices").arg(polygon_.size()), 4000);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::toggleROISelection() {
    if (btn_roi_->isChecked()) {
        if (polygon_.empty()) {
            QMessageBox::warning(this, "Warning", "Compute hull first.");
            btn_roi_->setChecked(false);
            return;
        }
        btn_obstacle_->setChecked(false);
        plot_->startROISelection();
        lbl_roi_->setText("ROI: selecting points...");
    } else {
        plot_->cancelSelection();
    }
}

void CoverageGUI::clearROI() {
    roi_polygon_.clear();
    plot_->clearROI();
    lbl_roi_->setText("ROI: none");
    setStatus("ROI cleared", 4000);
    refreshPlot();
}

void CoverageGUI::toggleObstacleSelection() {
    if (btn_obstacle_->isChecked()) {
        if (polygon_.empty()) {
            QMessageBox::warning(this, "Warning", "Compute hull first.");
            btn_obstacle_->setChecked(false);
            return;
        }
        btn_roi_->setChecked(false);
        plot_->startObstacleSelection();
        setStatus("Drawing obstacle polygon...");
    } else {
        plot_->cancelSelection();
    }
}

void CoverageGUI::clearObstacles() {
    obstacles_.clear();
    plot_->clearObstacles();
    lbl_obstacles_->setText("Obstacles: 0");
    setStatus("Obstacles cleared", 4000);
    refreshPlot();
}

void CoverageGUI::undoSelectionPoint() {
    plot_->undoLastPoint();
}

void CoverageGUI::finishSelection() {
    plot_->finishSelection();
}

void CoverageGUI::onROISelected(const Polygon2D& roi) {
    roi_polygon_ = roi;
    btn_roi_->setChecked(false);
    lbl_roi_->setText(QString("ROI: %1 vertices").arg(roi.size()));
    setStatus("ROI selected", 4000);
    refreshPlot();
}

void CoverageGUI::onObstacleSelected(const Polygon2D& obstacle) {
    obstacles_.push_back(obstacle);
    btn_obstacle_->setChecked(false);
    lbl_obstacles_->setText(QString("Obstacles: %1").arg(obstacles_.size()));
    setStatus(QString("Obstacle added (total: %1)").arg(obstacles_.size()), 4000);
    refreshPlot();
}

void CoverageGUI::onSelectionCancelled() {
    btn_roi_->setChecked(false);
    btn_obstacle_->setChecked(false);
    if (roi_polygon_.empty()) {
        lbl_roi_->setText("ROI: none");
    }
    setStatus("Selection cancelled", 3000);
}

void CoverageGUI::buildField() {
    // Just used for validation - actual field is built in generate functions
    if (polygon_.empty()) {
        QMessageBox::warning(this, "Warning", "Compute hull first.");
        return;
    }
    setStatus("Field ready for coverage generation", 4000);
}

void CoverageGUI::generateSwaths() {
    if (polygon_.empty()) {
        QMessageBox::warning(this, "Warning", "Compute hull first.");
        return;
    }
    
    showProgress(true, "Generating swaths...");
    progress_bar_->setRange(0, 0);  // Indeterminate
    
    try {
        CoverageConfig cfg = currentConfig();
        // Pass obstacles to coverage generation
        const std::vector<Polygon2D>* obs_ptr = obstacles_.empty() ? nullptr : &obstacles_;
        CoverageResult result = generateCoverage(effectivePolygon(), cfg, nullptr, obs_ptr);
        
        if (!result.success) {
            QMessageBox::critical(this, "Error", QString::fromStdString(result.error_message));
        } else {
            swaths_ = result.swaths;
            route_.clear();
            path_.clear();
            refreshPlot();
            setStatus(QString("Generated %1 swaths").arg(swaths_.size()), 4000);
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::generateRoute() {
    if (polygon_.empty()) {
        QMessageBox::warning(this, "Warning", "Compute hull first.");
        return;
    }
    
    showProgress(true, "Generating route...");
    progress_bar_->setRange(0, 0);
    
    try {
        CoverageConfig cfg = currentConfig();
        // Pass obstacles to coverage generation
        const std::vector<Polygon2D>* obs_ptr = obstacles_.empty() ? nullptr : &obstacles_;
        CoverageResult result = generateCoverage(effectivePolygon(), cfg, nullptr, obs_ptr);
        
        if (!result.success) {
            QMessageBox::critical(this, "Error", QString::fromStdString(result.error_message));
        } else {
            swaths_ = result.swaths;
            route_ = result.route;
            path_.clear();
            refreshPlot();
            setStatus(QString("Generated route with %1 waypoints").arg(route_.size()), 4000);
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::generatePath() {
    if (polygon_.empty()) {
        QMessageBox::warning(this, "Warning", "Compute hull first.");
        return;
    }
    
    showProgress(true, "Generating path...");
    progress_bar_->setRange(0, 0);
    
    try {
        CoverageConfig cfg = currentConfig();
        // Pass obstacles to coverage generation
        const std::vector<Polygon2D>* obs_ptr = obstacles_.empty() ? nullptr : &obstacles_;
        CoverageResult result = generateCoverage(effectivePolygon(), cfg, nullptr, obs_ptr);
        
        if (!result.success) {
            QMessageBox::critical(this, "Error", QString::fromStdString(result.error_message));
        } else {
            swaths_ = result.swaths;
            route_ = result.route;
            path_ = result.path;
            refreshPlot();
            setStatus(QString("Generated path with %1 states").arg(path_.size()), 4000);
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
}

void CoverageGUI::clearCoverage() {
    swaths_.clear();
    route_.clear();
    path_.clear();
    plot_->clearSwaths();
    plot_->clearRoute();
    plot_->clearPath();
    setStatus("Coverage cleared", 4000);
    refreshPlot();
}

void CoverageGUI::exportPathCSV() {
    // Determine which path to export based on mode
    PathStateList export_path;
    QString mode_label;
    
    if (isCustomModeActive()) {
        if (custom_waypoints_.empty()) {
            QMessageBox::warning(this, "Warning", "No custom waypoints to export.");
            return;
        }
        // Convert custom waypoints to PathStateList for export
        export_path.reserve(custom_waypoints_.size());
        for (const auto& pt : custom_waypoints_) {
            PathState ps;
            ps.point = pt;
            ps.heading = 0;
            ps.vx = 0;
            ps.vy = 0;
            export_path.push_back(ps);
        }
        mode_label = "custom";
    } else {
        if (path_.empty()) {
            QMessageBox::warning(this, "Warning", "Generate F2C path first.");
            return;
        }
        export_path = path_;
        mode_label = "F2C";
    }
    
    QString filename = QFileDialog::getSaveFileName(this, "Save Path CSV", "", "CSV (*.csv)");
    if (filename.isEmpty()) return;
    
    if (!filename.toLower().endsWith(".csv")) {
        filename += ".csv";
    }
    
    if (savePathToCSV(export_path, filename.toStdString())) {
        QMessageBox::information(this, "Export", 
                                QString("Saved %1 %2 waypoints").arg(export_path.size()).arg(mode_label));
        setStatus("Path exported", 4000);
    } else {
        QMessageBox::critical(this, "Error", "Failed to save file");
    }
}

void CoverageGUI::setupRobotTrackingSubscription() {
    fastlio_sub_.reset();
    
    if (!ros_node_) {
        updateRobotStatusLabel(false);
        return;
    }
    
    QString topic_qt = robot_odom_topic_.trimmed();
    if (topic_qt.isEmpty()) {
        updateRobotStatusLabel(false);
        return;
    }
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
    std::string topic = topic_qt.toStdString();
    std::cout << "[F2C GUI] Subscribing to robot odom topic: " << topic << std::endl;
    fastlio_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
        topic, qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            PathState state;
            state.point = {msg->pose.pose.position.x, msg->pose.pose.position.y};
            state.heading = yaw;
            state.vx = std::cos(yaw);
            state.vy = std::sin(yaw);
            
            {
                std::lock_guard<std::mutex> lock(robot_pose_mutex_);
                robot_pose_state_ = state;
                
                if (robot_trail_.empty() ||
                    std::hypot(robot_trail_.back().x - state.point.x,
                               robot_trail_.back().y - state.point.y) > 0.03) {
                    robot_trail_.push_back(state.point);
                    if (robot_trail_.size() > robot_trail_max_points_) {
                        robot_trail_.erase(
                            robot_trail_.begin(),
                            robot_trail_.begin() + (robot_trail_.size() - robot_trail_max_points_));
                    }
                }
                
                last_robot_update_ = std::chrono::steady_clock::now();
            }
            
            QMetaObject::invokeMethod(this, [this]() {
                updateCustomWaypointStatus();
                
                // Throttle plot refresh to reduce CPU usage at high odom rates
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_plot_refresh_).count();
                if (elapsed >= kPlotRefreshIntervalMs) {
                    last_plot_refresh_ = now;
                    refreshPlot();
                }
            }, Qt::QueuedConnection);
        });
    
    updateRobotStatusLabel(false);
}

void CoverageGUI::updateRobotStatusLabel(bool has_fix) {
    if (!lbl_robot_status_) {
        return;
    }
    
    QString topic = robot_odom_topic_.isEmpty() ? "(topic not set)" : robot_odom_topic_;
    if (has_fix) {
        lbl_robot_status_->setText(QString("Robot: tracking (%1)").arg(topic));
        lbl_robot_status_->setStyleSheet("color: #2e7d32; font-size: 10px;");
    } else {
        lbl_robot_status_->setText(QString("Robot: waiting for %1...").arg(topic));
        lbl_robot_status_->setStyleSheet("color: #a66f00; font-size: 10px;");
    }
}

void CoverageGUI::scheduleFitToView() {
    fit_view_pending_ = true;
}

void CoverageGUI::publishWaypoints() {
    // Check ROS2 availability first
    if (!ros_initialized_) {
        QMessageBox::warning(this, "ROS2 Unavailable", 
            "ROS2 is not initialized. Cannot publish waypoints.\n\n"
            "Check your network configuration and CycloneDDS settings.");
        return;
    }
    if (!waypoint_pub_) {
        QMessageBox::warning(this, "ROS2 Unavailable", "Waypoint publisher is not ready yet.");
        return;
    }
    
    std_msgs::msg::Float64MultiArray msg;
    size_t waypoint_count = 0;
    
    if (isCustomModeActive()) {
        // Custom path mode
        if (custom_waypoints_.size() < 2) {
            QMessageBox::warning(this, "No Path", "Add at least two custom waypoints before publishing.");
            return;
        }
        
        msg.data.reserve(custom_waypoints_.size() * 2);
        for (const auto& pt : custom_waypoints_) {
            msg.data.push_back(pt.x);
            msg.data.push_back(pt.y);
        }
        waypoint_count = custom_waypoints_.size();
        
        // Reset visited status for tracking
        custom_waypoints_visited_.assign(custom_waypoints_.size(), false);
        refreshCustomPathUI();
        
        std::cout << "[F2C GUI] Published " << waypoint_count << " custom waypoints to /f2c_waypoints topic" << std::endl;
    } else {
        // F2C coverage mode
        if (path_.empty()) {
            QMessageBox::warning(this, "No Path", "Generate a coverage path first before publishing waypoints.");
            return;
        }
        
        // Remove consecutive duplicates to avoid sending repeated points
        PathStateList deduped_path = dedupePathStates(path_);
        
        msg.data.reserve(deduped_path.size() * 2);
        for (const auto& state : deduped_path) {
            msg.data.push_back(state.point.x);
            msg.data.push_back(state.point.y);
        }
        waypoint_count = deduped_path.size();
        
        std::cout << "[F2C GUI] Published " << waypoint_count << " F2C waypoints to /f2c_waypoints topic" << std::endl;
    }
    
    // Publish to ROS2 topic
    waypoint_pub_->publish(msg);
    waypoints_published_ = true;
    
    // Update status and enable navigation button
    setStatus(QString("✅ Published %1 waypoints to robot").arg(waypoint_count), 5000);
    
    if (btn_start_navigation_) {
        btn_start_navigation_->setEnabled(true);
    }
}

void CoverageGUI::publishCustomPath() {
    // Redirect to unified publish function
    publishWaypoints();
}

void CoverageGUI::onPathModeChanged() {
    bool custom_mode = isCustomModeActive();
    
    // Show/hide control panels
    if (f2c_controls_widget_) {
        f2c_controls_widget_->setVisible(!custom_mode);
    }
    if (custom_controls_widget_) {
        custom_controls_widget_->setVisible(custom_mode);
    }
    
    // Disable drawing if switching away from custom mode
    setCustomModeActive(custom_mode);
    refreshCustomPathUI();
    refreshPlot();
}

bool CoverageGUI::isCustomModeActive() const {
    return radio_mode_custom_ && radio_mode_custom_->isChecked();
}

void CoverageGUI::setCustomModeActive(bool active) {
    if (!plot_) {
        return;
    }
    
    if (!active && custom_draw_enabled_) {
        custom_draw_enabled_ = false;
        if (btn_custom_draw_) {
            QSignalBlocker blocker(btn_custom_draw_);
            btn_custom_draw_->setChecked(false);
        }
    }
    
    plot_->setCustomDrawMode(active && custom_draw_enabled_);
    plot_->setShowCustomPath(active && !custom_waypoints_.empty());
}

void CoverageGUI::onPlotCustomWaypoint(const Point2D& point) {
    if (!isCustomModeActive() || !custom_draw_enabled_) {
        return;
    }
    custom_waypoints_.push_back(point);
    custom_waypoints_visited_.push_back(false);
    refreshCustomPathUI();
}

void CoverageGUI::refreshCustomPathUI() {
    custom_waypoints_visited_.resize(custom_waypoints_.size(), false);
    
    if (list_custom_points_) {
        list_custom_points_->clear();
        for (size_t i = 0; i < custom_waypoints_.size(); ++i) {
            const auto& pt = custom_waypoints_[i];
            bool visited = custom_waypoints_visited_[i];
            QString text = QString("#%1 (%2, %3) %4")
                .arg(i + 1)
                .arg(pt.x, 0, 'f', 2)
                .arg(pt.y, 0, 'f', 2)
                .arg(visited ? "✓ reached" : "→ pending");
            auto* item = new QListWidgetItem(text);
            item->setForeground(visited ? QColor("#2e7d32") : QColor("#006064"));
            list_custom_points_->addItem(item);
        }
    }
    
    size_t next_idx = custom_waypoints_.size();
    for (size_t i = 0; i < custom_waypoints_.size(); ++i) {
        if (!custom_waypoints_visited_[i]) {
            next_idx = i;
            break;
        }
    }
    
    if (lbl_custom_status_) {
        if (custom_waypoints_.empty()) {
            lbl_custom_status_->setText("No custom waypoints yet.");
        } else if (next_idx >= custom_waypoints_.size()) {
            lbl_custom_status_->setText(QString("Waypoints: %1 (all reached)").arg(custom_waypoints_.size()));
        } else {
            lbl_custom_status_->setText(
                QString("Waypoints: %1 | Next target: #%2")
                .arg(custom_waypoints_.size())
                .arg(next_idx + 1));
        }
    }
    
    if (btn_custom_undo_) {
        btn_custom_undo_->setEnabled(!custom_waypoints_.empty());
    }
    if (btn_custom_clear_) {
        btn_custom_clear_->setEnabled(!custom_waypoints_.empty());
    }
    
    plot_->setCustomPath(custom_waypoints_, custom_waypoints_visited_);
    plot_->setShowCustomPath(isCustomModeActive() && !custom_waypoints_.empty());
}

void CoverageGUI::undoCustomWaypoint() {
    if (custom_waypoints_.empty()) {
        return;
    }
    custom_waypoints_.pop_back();
    if (!custom_waypoints_visited_.empty()) {
        custom_waypoints_visited_.resize(custom_waypoints_.size());
    }
    refreshCustomPathUI();
}

void CoverageGUI::clearCustomWaypoints() {
    if (custom_waypoints_.empty()) {
        return;
    }
    custom_waypoints_.clear();
    custom_waypoints_visited_.clear();
    refreshCustomPathUI();
}

void CoverageGUI::updateCustomWaypointStatus() {
    if (custom_waypoints_.empty()) {
        return;
    }
    
    std::optional<PathState> pose_copy;
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        pose_copy = robot_pose_state_;
    }
    
    if (!pose_copy.has_value()) {
        return;
    }
    
    bool updated = false;
    for (size_t i = 0; i < custom_waypoints_.size(); ++i) {
        if (i >= custom_waypoints_visited_.size()) {
            custom_waypoints_visited_.resize(custom_waypoints_.size(), false);
        }
        if (custom_waypoints_visited_[i]) {
            continue;
        }
        
        const auto& target = custom_waypoints_[i];
        double dist = std::hypot(pose_copy->point.x - target.x,
                                 pose_copy->point.y - target.y);
        if (dist <= custom_waypoint_reach_tol_) {
            custom_waypoints_visited_[i] = true;
            updated = true;
            continue;
        }
        break;  // Waypoints are sequential
    }
    
    if (updated) {
        refreshCustomPathUI();
    }
}

void CoverageGUI::startNavigation() {
    if (!waypoints_published_) {
        QMessageBox::warning(this, "Waypoints Not Published", "Please publish waypoints first.");
        return;
    }

    if (!ros_initialized_) {
        QMessageBox::warning(this, "ROS2 Unavailable", 
            "ROS2 is not initialized. Cannot start navigation.");
        return;
    }

    // Publish empty message to signal start of navigation
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {0.0};  // Special signal value
    waypoint_pub_->publish(msg);

    setStatus("🚀 Navigation started!", 3000);
    std::cout << "[F2C GUI] Sent navigation start signal" << std::endl;
}

// Helper: find closest point on a line segment to a given point
static Point2D closestPointOnSegment(const Point2D& p, const Point2D& a, const Point2D& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    
    if (len_sq < 1e-10) {
        // Degenerate segment
        return a;
    }
    
    // Project p onto line, clamped to segment [0,1]
    double t = std::max(0.0, std::min(1.0,
        ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq));
    
    return Point2D{a.x + t * dx, a.y + t * dy};
}

// Helper: find closest point on a polyline path to a given point
static std::pair<Point2D, double> closestPointOnPath(const Point2D& p, const std::vector<Point2D>& path) {
    if (path.empty()) {
        return {p, std::numeric_limits<double>::max()};
    }
    if (path.size() == 1) {
        double dist = std::hypot(p.x - path[0].x, p.y - path[0].y);
        return {path[0], dist};
    }
    
    Point2D best_point = path[0];
    double best_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 1; i < path.size(); ++i) {
        Point2D closest = closestPointOnSegment(p, path[i-1], path[i]);
        double dist = std::hypot(p.x - closest.x, p.y - closest.y);
        if (dist < best_dist) {
            best_dist = dist;
            best_point = closest;
        }
    }
    
    return {best_point, best_dist};
}

void CoverageGUI::computeReprojectionError() {
    // Determine which waypoints to use based on current mode
    std::vector<Point2D> path_points;
    if (isCustomModeActive()) {
        path_points = custom_waypoints_;
    } else {
        // Use F2C path waypoints
        for (const auto& state : path_) {
            path_points.push_back(state.point);
        }
    }
    
    if (path_points.size() < 2) {
        QMessageBox::warning(this, "No Path", 
            "Need at least 2 waypoints. Generate or draw a path first.");
        return;
    }
    
    // Get current robot trail
    std::vector<Point2D> trail;
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        trail = robot_trail_;
    }
    
    if (trail.size() < 2) {
        QMessageBox::warning(this, "No Trail", 
            "Robot trail is empty. Move the robot first.");
        return;
    }
    
    // Parameters
    const double sample_interval = 0.05;  // 5cm sampling along trail
    const double max_association_dist = 1.0;  // 1m threshold
    
    reproj_lines_.clear();
    
    // Resample the trail at 5cm intervals
    std::vector<Point2D> sampled_trail;
    sampled_trail.push_back(trail[0]);
    double accumulated_dist = 0;
    
    for (size_t i = 1; i < trail.size(); ++i) {
        double seg_dist = std::hypot(trail[i].x - trail[i-1].x, 
                                      trail[i].y - trail[i-1].y);
        accumulated_dist += seg_dist;
        
        // Add sample points at each 5cm interval
        while (accumulated_dist >= sample_interval) {
            // Interpolate position at sample point
            double overshoot = accumulated_dist - sample_interval;
            double ratio = (seg_dist > 1e-6) ? (seg_dist - overshoot) / seg_dist : 1.0;
            
            Point2D sample;
            sample.x = trail[i-1].x + ratio * (trail[i].x - trail[i-1].x);
            sample.y = trail[i-1].y + ratio * (trail[i].y - trail[i-1].y);
            sampled_trail.push_back(sample);
            
            accumulated_dist -= sample_interval;
        }
    }
    
    // For each sampled trail point, find closest point on planned path
    int sample_index = 0;
    for (const auto& trail_pt : sampled_trail) {
        auto [closest_path_pt, dist] = closestPointOnPath(trail_pt, path_points);
        
        // Only include if within threshold
        if (dist <= max_association_dist) {
            ReprojectionLine line;
            line.waypoint = closest_path_pt;  // Point on planned path
            line.traversed = trail_pt;         // Point on robot trail
            line.error_m = dist;
            line.waypoint_index = sample_index;
            reproj_lines_.push_back(line);
        }
        sample_index++;
    }
    
    if (reproj_lines_.empty()) {
        QMessageBox::information(this, "No Match", 
            "No trail points found within 1m of the planned path.\n"
            "Make sure the robot has traversed near the path.");
        return;
    }
    
    // Compute statistics
    double total_error = 0;
    double max_error = 0;
    for (const auto& line : reproj_lines_) {
        total_error += line.error_m;
        max_error = std::max(max_error, line.error_m);
    }
    double avg_error = total_error / reproj_lines_.size();
    
    // Update plot
    plot_->setReprojectionLines(reproj_lines_);
    
    // Update status
    QString status = QString("Reprojection: %1 samples (5cm), avg=%2 cm, max=%3 cm")
        .arg(reproj_lines_.size())
        .arg(avg_error * 100, 0, 'f', 1)
        .arg(max_error * 100, 0, 'f', 1);
    if (lbl_reproj_status_) {
        lbl_reproj_status_->setText(status);
        lbl_reproj_status_->setStyleSheet("color: #1565c0; font-size: 10px;");
    }
    setStatus(status, 5000);
    
    std::cout << "[F2C GUI] Reprojection error computed: " << reproj_lines_.size() 
              << " samples (5cm), avg=" << (avg_error * 100) << " cm, max=" << (max_error * 100) << " cm" << std::endl;
}

void CoverageGUI::clearReprojectionError() {
    reproj_lines_.clear();
    plot_->clearReprojectionLines();
    if (lbl_reproj_status_) {
        lbl_reproj_status_->setText("No reprojection computed");
        lbl_reproj_status_->setStyleSheet("color: #666; font-size: 10px;");
    }
    setStatus("Reprojection error cleared", 3000);
}

void CoverageGUI::tryReconnectROS2() {
    // Already connected - stop the timer
    if (ros_initialized_) {
        ros_reconnect_timer_->stop();
        return;
    }
    
    std::cout << "[F2C GUI] Attempting ROS2 reconnection..." << std::endl;
    
    try {
        // Try to initialize ROS2
        ros_node_ = rclcpp::Node::make_shared("f2c_coverage_gui");
        waypoint_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/f2c_waypoints", 10);
        setupRobotTrackingSubscription();

        // Start ROS2 spinning in background thread
        ros_thread_ = std::thread([this]() {
            rclcpp::spin(ros_node_);
        });
        
        ros_initialized_ = true;
        
        // Success! Stop the reconnection timer
        ros_reconnect_timer_->stop();
        
        setStatus(QString("✅ ROS2 connected via %1!").arg(dds_profile_.toUpper()), 5000);
        std::cout << "[F2C GUI] ROS2 reconnection successful!" << std::endl;
        
    } catch (const std::exception& e) {
        // Still not available - timer will try again
        setStatus("ROS2 unavailable - retrying...", 4500);
    }
}

QString CoverageGUI::currentDdsConfigPath() const {
    if (dds_profile_ == "wifi") {
        return dds_wifi_config_path_;
    }
    return dds_rf_config_path_;
}

void CoverageGUI::onDdsProfileChanged() {
    // Determine which profile is now selected
    QString new_profile = radio_dds_rf_->isChecked() ? "rf" : "wifi";
    
    // Skip if unchanged
    if (new_profile == dds_profile_) {
        return;
    }
    
    dds_profile_ = new_profile;
    
    // Persist the selection
    QSettings settings("PilotControl", "F2CCoveragePlanner");
    settings.setValue("dds_profile", dds_profile_);
    
    // Update status label
    QString config_path = currentDdsConfigPath();
    if (QFile::exists(config_path)) {
        lbl_dds_status_->setText("✓ Config: ~/" + QFileInfo(config_path).fileName());
        lbl_dds_status_->setStyleSheet("color: green; font-size: 10px;");
    } else {
        lbl_dds_status_->setText("⚠ Config not found: ~/" + QFileInfo(config_path).fileName());
        lbl_dds_status_->setStyleSheet("color: orange; font-size: 10px;");
    }
    
    std::cout << "[F2C GUI] DDS profile changed to: " << dds_profile_.toStdString() << std::endl;
    
    // Update environment variable
    qputenv("CYCLONEDDS_URI", config_path.toUtf8());
    
    // Reinitialize ROS2 with new DDS config
    reinitializeROS2();
}

void CoverageGUI::reinitializeROS2() {
    setStatus(QString("Switching to %1 network...").arg(dds_profile_.toUpper()));
    
    // Stop reconnection timer if running
    ros_reconnect_timer_->stop();
    
    // Shutdown existing ROS2 connection if any
    if (ros_initialized_) {
        std::cout << "[F2C GUI] Shutting down ROS2 for profile switch..." << std::endl;
        fastlio_sub_.reset();
        
        // Stop the spin thread by shutting down the node's context
        if (ros_node_) {
            rclcpp::shutdown();
            
            // Wait for thread to finish
            if (ros_thread_.joinable()) {
                ros_thread_.join();
            }
            
            // Reset pointers
            waypoint_pub_.reset();
            ros_node_.reset();
        }
        
        ros_initialized_ = false;
        waypoints_published_ = false;
        
        // Reinitialize rclcpp for the new configuration
        // Note: rclcpp::init should be called again after shutdown
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
    
    // Try to initialize with new config
    QString config_path = currentDdsConfigPath();
    if (!QFile::exists(config_path)) {
        setStatus(QString("⚠ %1 config not found - ROS2 may fail").arg(dds_profile_.toUpper()), 5000);
    }
    
    try {
        ros_node_ = rclcpp::Node::make_shared("f2c_coverage_gui");
        waypoint_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/f2c_waypoints", 10);
        setupRobotTrackingSubscription();

        ros_thread_ = std::thread([this]() {
            rclcpp::spin(ros_node_);
        });
        
        ros_initialized_ = true;
        setStatus(QString("✅ ROS2 connected via %1").arg(dds_profile_.toUpper()), 5000);
        std::cout << "[F2C GUI] ROS2 reinitialized with " << dds_profile_.toStdString() << " profile" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[F2C GUI] ROS2 reinit failed: " << e.what() << std::endl;
        setStatus(QString("ROS2 unavailable on %1 - retrying...").arg(dds_profile_.toUpper()));
        ros_reconnect_timer_->start();
    }
}

} // namespace f2c_cpp

