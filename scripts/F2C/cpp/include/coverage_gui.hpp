/**
 * @file coverage_gui.hpp
 * @brief Qt6 GUI for coverage planning
 * 
 * This is the C++ equivalent of f2c_gui.py
 */

#pragma once

#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QProgressBar>
#include <QStatusBar>
#include <QScrollArea>
#include <QSplitter>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

#include "coverage_pipeline.hpp"

namespace f2c_cpp {

// =============================================================================
// Custom Plot Widget
// =============================================================================

class PlotWidget : public QWidget {
    Q_OBJECT

public:
    explicit PlotWidget(QWidget* parent = nullptr);

    // Data setters
    void setPoints(const std::vector<Point2D>& points);
    void setPolygon(const Polygon2D& poly);
    void setROI(const Polygon2D& roi);
    void setObstacles(const std::vector<Polygon2D>& obstacles);
    void setSwaths(const SwathList& swaths);
    void setRoute(const PathStateList& route);
    void setPath(const PathStateList& path);
    
    // Clear functions
    void clearAll();
    void clearPoints();
    void clearPolygon();
    void clearROI();
    void clearObstacles();
    void clearSwaths();
    void clearRoute();
    void clearPath();
    
    // View control
    void resetView();
    void zoomIn();
    void zoomOut();
    
    // ROI/Obstacle selection
    void startROISelection();
    void startObstacleSelection();
    void finishSelection();
    void cancelSelection();
    void undoLastPoint();
    
    bool isSelecting() const { return selecting_; }
    Polygon2D getSelectedPolygon() const;

signals:
    void roiSelected(const Polygon2D& roi);
    void obstacleSelected(const Polygon2D& obstacle);
    void selectionCancelled();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    // Data
    std::vector<Point2D> points_;
    Polygon2D polygon_;
    Polygon2D roi_;
    std::vector<Polygon2D> obstacles_;
    SwathList swaths_;
    PathStateList route_;
    PathStateList path_;
    
    // View transform
    double scale_ = 1.0;
    double offset_x_ = 0.0;
    double offset_y_ = 0.0;
    double data_min_x_ = 0, data_max_x_ = 1;
    double data_min_y_ = 0, data_max_y_ = 1;
    
    // Selection state
    bool selecting_ = false;
    bool selecting_roi_ = false;  // true = ROI, false = obstacle
    std::vector<Point2D> selection_points_;
    QPointF cursor_pos_;
    
    // Panning
    bool panning_ = false;
    QPoint pan_start_;
    double pan_offset_x_, pan_offset_y_;
    
    // Helpers
    QPointF worldToScreen(const Point2D& p) const;
    Point2D screenToWorld(const QPointF& p) const;
    void updateDataBounds();
    void fitToData();
};

// =============================================================================
// Main GUI Window
// =============================================================================

class CoverageGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit CoverageGUI(QWidget* parent = nullptr);
    ~CoverageGUI() override = default;

private slots:
    // File operations
    void loadPointCloud();
    
    // Processing
    void applyHeightCrop();
    void applyDownsample();
    void computeHull();
    void simplifyPolygon();
    
    // ROI / Obstacles
    void toggleROISelection();
    void clearROI();
    void toggleObstacleSelection();
    void clearObstacles();
    void undoSelectionPoint();
    void finishSelection();
    
    // Coverage
    void buildField();
    void generateSwaths();
    void generateRoute();
    void generatePath();
    void clearCoverage();
    
    // Export
    void exportPathCSV();
    void publishWaypoints();
    void startNavigation();
    
    // Callbacks
    void onROISelected(const Polygon2D& roi);
    void onObstacleSelected(const Polygon2D& obstacle);
    void onSelectionCancelled();
    
    // UI updates
    void updateDownsampleUI(const QString& method);

private:
    void setupUI();
    void setupConnections();
    
    // UI building helpers
    QGroupBox* buildFileControls();
    QGroupBox* buildHeightControls();
    QGroupBox* buildDownsampleControls();
    QGroupBox* buildHullControls();
    QGroupBox* buildSimplifyControls();
    QGroupBox* buildCoverageControls();
    QGroupBox* buildExportControls();
    
    // Status/Progress
    void setStatus(const QString& text, int timeout_ms = 0);
    void showProgress(bool show, const QString& text = "");
    void updateProgress(int percent, const QString& text = "");
    
    // Get current configuration
    CoverageConfig currentConfig() const;
    
    // Refresh plot
    void refreshPlot();
    
    // Apply effective polygon (with ROI/obstacles)
    Polygon2D effectivePolygon() const;

private:
    // Main widgets
    PlotWidget* plot_;
    QStatusBar* status_bar_;
    QProgressBar* progress_bar_;
    
    // File controls
    QLabel* lbl_file_;
    
    // Height controls
    QDoubleSpinBox* spin_z_band_;
    
    // Downsample controls
    QComboBox* combo_downsample_;
    QGroupBox* group_random_;
    QGroupBox* group_voxel_;
    QGroupBox* group_stat_;
    QSpinBox* spin_max_points_;
    QDoubleSpinBox* spin_voxel_;
    QSpinBox* spin_mean_k_;
    QDoubleSpinBox* spin_std_ratio_;
    
    // Hull controls
    QComboBox* combo_hull_method_;
    QDoubleSpinBox* spin_alpha_;
    
    // Simplify controls
    QDoubleSpinBox* spin_simplify_;
    
    // Coverage controls
    QDoubleSpinBox* spin_swath_;
    QDoubleSpinBox* spin_headland_;
    QDoubleSpinBox* spin_turn_;
    QCheckBox* chk_auto_align_;
    QRadioButton* radio_long_;
    QRadioButton* radio_perp_;
    QComboBox* combo_route_pattern_;
    QComboBox* combo_path_planner_;
    QCheckBox* chk_decomposition_;
    QComboBox* combo_decomp_type_;
    QCheckBox* chk_axial_turns_;
    
    // ROI controls
    QPushButton* btn_roi_;
    QPushButton* btn_roi_clear_;
    QPushButton* btn_roi_finish_;
    QPushButton* btn_roi_undo_;
    QLabel* lbl_roi_;
    
    // Obstacle controls
    QPushButton* btn_obstacle_;
    QPushButton* btn_obstacle_clear_;
    QLabel* lbl_obstacles_;
    
    // State
    PointCloudPtr pcd_points_;
    PointCloudPtr filtered_points_;
    std::vector<Point2D> xy_2d_;
    Polygon2D polygon_;
    Polygon2D roi_polygon_;
    std::vector<Polygon2D> obstacles_;
    SwathList swaths_;
    PathStateList route_;
    PathStateList path_;
    QString loaded_file_;

    // ROS2 integration for waypoint publishing
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr waypoint_pub_;
    std::thread ros_thread_;
    bool waypoints_published_;
    bool ros_initialized_;
};

} // namespace f2c_cpp

