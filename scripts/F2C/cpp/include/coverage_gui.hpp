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
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QProgressBar>
#include <QStatusBar>
#include <QScrollArea>
#include <QListWidget>
#include <QSplitter>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <QDir>
#include <QDate>
#include <QSettings>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QAction>
#include <QShortcut>
#include <QtConcurrent>
#include <QFutureWatcher>
#include <QToolBox>
#include <QButtonGroup>
#include <QDockWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>
#include <optional>
#include <mutex>
#include <chrono>

// GStreamer for video streaming
#include <gst/gst.h>
#include <gst/video/videooverlay.h>

#include "coverage_pipeline.hpp"

namespace f2c_cpp {

// =============================================================================
// Reprojection Error Line (waypoint vs traversed)
// =============================================================================

struct ReprojectionLine {
    Point2D waypoint;      // The target waypoint
    Point2D traversed;     // Closest traversed point
    double error_m;        // Error in meters
    int waypoint_index;    // Index in the path
};

// =============================================================================
// Coverage Statistics
// =============================================================================

struct CoverageStats {
    double path_length_m = 0;       // Total path length in meters
    double coverage_area_m2 = 0;    // Covered area in square meters
    double polygon_area_m2 = 0;     // Field/ROI area
    double coverage_percent = 0;    // Coverage percentage
    int num_swaths = 0;             // Number of swaths
    int num_turns = 0;              // Number of turns
    int num_waypoints = 0;          // Number of waypoints
    double estimated_time_min = 0;  // Estimated time at given speed
    double overlap_percent = 0;     // Swath overlap percentage
    
    bool isValid() const { return path_length_m > 0; }
};

// =============================================================================
// Video Stream Widget (GStreamer embedded)
// =============================================================================

class VideoStreamWidget : public QWidget {
    Q_OBJECT

public:
    explicit VideoStreamWidget(QWidget* parent = nullptr);
    ~VideoStreamWidget();
    
    void startStream(int port);
    void stopStream();
    bool isPlaying() const { return playing_; }
    int currentPort() const { return current_port_; }

signals:
    void streamError(const QString& msg);
    void streamStarted();
    void streamStopped();

protected:
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

private slots:
    void pollBus();

private:
    GstElement* pipeline_ = nullptr;
    int current_port_ = 5600;
    bool playing_ = false;
    bool auto_start_on_show_ = false;
    QTimer* bus_poll_timer_ = nullptr;
    
    void setupPipeline(int port);
    void destroyPipeline();
    static GstBusSyncReply busSyncHandler(GstBus* bus, GstMessage* msg, gpointer data);
    static gboolean busCallback(GstBus* bus, GstMessage* msg, gpointer data);
};

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
    void setRobotPose(const std::optional<PathState>& pose);
    void setRobotTrail(const std::vector<Point2D>& trail);
    void setRobotMarkerSize(double size_meters);
    void setCustomPath(const std::vector<Point2D>& path,
                       const std::vector<bool>& visited);
    void setShowCustomPath(bool show);
    void setCustomDrawMode(bool enabled);
    void setReprojectionLines(const std::vector<ReprojectionLine>& lines);
    void clearReprojectionLines();
    int getHoveredReprojectionIndex() const { return hovered_reproj_index_; }
    
    // Rectangle drawing mode
    void startRectangleMode();
    void cancelRectangleMode();
    bool isDrawingRectangle() const { return drawing_rectangle_; }
    
    // Dark mode
    void setDarkMode(bool enabled);
    bool isDarkMode() const { return dark_mode_; }
    
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
    void customWaypointRequested(const Point2D& point);
    void rectangleCompleted(const Polygon2D& rect);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
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
    std::optional<PathState> robot_pose_;
    std::vector<Point2D> robot_trail_;
    double robot_marker_size_ = 0.6;
    std::vector<Point2D> custom_waypoints_;
    std::vector<bool> custom_waypoint_states_;
    bool show_custom_path_ = false;
    bool custom_draw_mode_ = false;
    
    // Reprojection error visualization
    std::vector<ReprojectionLine> reproj_lines_;
    int hovered_reproj_index_ = -1;  // -1 = none hovered
    
    // Rectangle drawing mode (3-click: corner1, corner2 defines edge, corner3 defines width)
    bool drawing_rectangle_ = false;
    std::vector<Point2D> rect_points_;  // 0-2 points during drawing
    
    // Dark mode
    bool dark_mode_ = false;
    
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
    double distanceToLineSegment(const QPointF& mouse, const QPointF& p1, const QPointF& p2) const;
};

// =============================================================================
// Main GUI Window
// =============================================================================

class CoverageGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit CoverageGUI(QWidget* parent = nullptr);
    ~CoverageGUI() override;

private slots:
    // File operations
    void loadPointCloud();
    void fetchLatestMapFromRobot();
    void loadPointCloudFromPath(const QString& path);
    
    // Processing
    void applyHeightCrop();
    void applyDownsample();
    void computeHull();
    void simplifyPolygon();
    
    // 3D Visualization
    void showPointCloud3D();
    
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
    void clearRobotTrail();
    void onPathModeChanged();
    
    // Callbacks
    void onROISelected(const Polygon2D& roi);
    void onObstacleSelected(const Polygon2D& obstacle);
    void onSelectionCancelled();
    
    // UI updates
    void updateDownsampleUI(const QString& method);
    
    // ROS2 reconnection
    void tryReconnectROS2();
    
    // DDS profile switching
    void onDdsProfileChanged();
    
    // Reprojection error analysis
    void computeReprojectionError();
    void clearReprojectionError();
    
    // Rectangle drawing
    void toggleRectangleMode();
    void onRectangleCompleted(const Polygon2D& rect);
    
    // Dark mode
    void toggleDarkMode();
    void applyTheme();
    
    // Coverage statistics
    void updateCoverageStats();
    CoverageStats computeStats() const;
    
    // Workflow steps
    void updateWorkflowSteps();
    void onWorkflowStepClicked(int step);
    
    // Layer visibility
    void updateLayerVisibility();
    
    // Video streaming
    void setupVideoPanel();
    void toggleVideoPanel();
    void onCameraToggled(bool right_selected);
    void playVideoStream();
    void stopVideoStream();
    void onCameraStatusReceived(const std_msgs::msg::String::SharedPtr msg);

private:
    void setupUI();
    void setupConnections();
    void setupRobotTrackingSubscription();
    
    // UI building helpers
    QGroupBox* buildFileControls();
    QGroupBox* buildHeightControls();
    QGroupBox* buildDownsampleControls();
    QGroupBox* buildHullControls();
    QGroupBox* buildSimplifyControls();
    QGroupBox* buildCoverageControls();
    QGroupBox* buildExportControls();
    QGroupBox* buildRobotTrackingControls();
    QGroupBox* buildPathPlanningControls();
    QWidget* buildF2CControls();
    QWidget* buildCustomPathControls();
    QGroupBox* buildCoverageStatsControls();
    QWidget* buildWorkflowIndicator();
    QWidget* buildLayerPanel();
    QWidget* buildQuickActionsBar();
    
    // Async point cloud loading
    void loadPointCloudAsync(const QString& path);
    void onPointCloudLoaded();
    
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
    void updateRobotStatusLabel(bool has_fix);
    void scheduleFitToView();
    void refreshCustomPathUI();
    void onPlotCustomWaypoint(const Point2D& point);
    void undoCustomWaypoint();
    void clearCustomWaypoints();
    void publishCustomPath();
    void updateCustomWaypointStatus();
    void setCustomModeActive(bool active);
    bool isCustomModeActive() const;

private:
    // Main widgets
    PlotWidget* plot_;
    QStatusBar* status_bar_;
    QProgressBar* progress_bar_;
    
    // File controls
    QLabel* lbl_file_;
    QLineEdit* txt_robot_ip_;
    QCheckBox* chk_show_robot_;
    QPushButton* btn_clear_robot_trail_;
    QLabel* lbl_robot_status_;
    QLineEdit* txt_robot_topic_;
    QDoubleSpinBox* spin_robot_marker_size_;
    
    // Path mode selector (F2C vs Custom)
    QRadioButton* radio_mode_f2c_ = nullptr;
    QRadioButton* radio_mode_custom_ = nullptr;
    QWidget* f2c_controls_widget_ = nullptr;
    QWidget* custom_controls_widget_ = nullptr;
    QPushButton* btn_custom_draw_ = nullptr;
    QPushButton* btn_custom_undo_ = nullptr;
    QPushButton* btn_custom_clear_ = nullptr;
    QListWidget* list_custom_points_ = nullptr;
    QLabel* lbl_custom_status_ = nullptr;
    QPushButton* btn_publish_waypoints_ = nullptr;
    QPushButton* btn_start_navigation_ = nullptr;
    
    // DDS profile controls
    QRadioButton* radio_dds_rf_;
    QRadioButton* radio_dds_wifi_;
    QLabel* lbl_dds_status_;
    
    // Height controls (Z range filtering relative to robot origin Z=0)
    QDoubleSpinBox* spin_z_min_;   // Minimum Z value (can be negative)
    QDoubleSpinBox* spin_z_max_;   // Maximum Z value
    
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
    std::thread ros_thread_;
    bool waypoints_published_;
    bool ros_initialized_;
    
    // ROS2 reconnection timer (only active when disconnected)
    QTimer* ros_reconnect_timer_;
    
    // Robot connection settings for map fetching
    QString robot_host_ = "192.168.168.101";
    QString robot_user_ = "roofus";
    QString robot_data_path_ = "/R_DATA";
    QString local_map_base_;  // Set to ~/Roofus_maps in constructor
    
    // CycloneDDS profile settings (user-agnostic paths)
    QString dds_profile_;           // "rf" or "wifi"
    QString dds_rf_config_path_;    // ~/rf_cyclonedds.xml
    QString dds_wifi_config_path_;  // ~/wifi_cyclonedds.xml
    
    // Helper to get current DDS config path
    QString currentDdsConfigPath() const;
    
    // Helper to shutdown and reinitialize ROS2 with new DDS config
    void reinitializeROS2();
    
    // Robot tracking state
    mutable std::mutex robot_pose_mutex_;
    std::optional<PathState> robot_pose_state_;
    std::vector<Point2D> robot_trail_;
    size_t robot_trail_max_points_ = 5000;
    std::chrono::steady_clock::time_point last_robot_update_;
    QString robot_odom_topic_ = "/Odometry_tilt_corrected_diff";
    double robot_marker_size_m_ = 0.6;
    bool fit_view_pending_ = false;
    bool custom_draw_enabled_ = false;
    std::vector<Point2D> custom_waypoints_;
    std::vector<bool> custom_waypoints_visited_;
    double custom_waypoint_reach_tol_ = 0.10;
    
    // Throttle plot refresh to avoid excessive repaints from high-frequency odom
    std::chrono::steady_clock::time_point last_plot_refresh_;
    static constexpr int kPlotRefreshIntervalMs = 50;  // ~20 Hz max
    
    // Reprojection error analysis
    QPushButton* btn_compute_reproj_ = nullptr;
    QPushButton* btn_clear_reproj_ = nullptr;
    QLabel* lbl_reproj_status_ = nullptr;
    std::vector<ReprojectionLine> reproj_lines_;
    
    // Rectangle drawing tool
    QPushButton* btn_rectangle_ = nullptr;
    
    // Dark mode
    bool dark_mode_ = false;
    QPushButton* btn_dark_mode_ = nullptr;
    
    // Coverage statistics widgets
    QGroupBox* stats_group_ = nullptr;
    QLabel* lbl_stats_path_length_ = nullptr;
    QLabel* lbl_stats_area_ = nullptr;
    QLabel* lbl_stats_coverage_ = nullptr;
    QLabel* lbl_stats_swaths_ = nullptr;
    QLabel* lbl_stats_turns_ = nullptr;
    QLabel* lbl_stats_waypoints_ = nullptr;
    QLabel* lbl_stats_time_ = nullptr;
    QDoubleSpinBox* spin_robot_speed_ = nullptr;
    CoverageStats current_stats_;
    
    // Async loading
    QFutureWatcher<PointCloudPtr>* pcd_watcher_ = nullptr;
    QString pending_load_path_;
    
    // Workflow steps indicator
    QWidget* workflow_widget_ = nullptr;
    std::vector<QPushButton*> workflow_btns_;
    int current_workflow_step_ = 0;
    
    // Layer visibility panel
    QCheckBox* chk_layer_points_ = nullptr;
    QCheckBox* chk_layer_polygon_ = nullptr;
    QCheckBox* chk_layer_roi_ = nullptr;
    QCheckBox* chk_layer_obstacles_ = nullptr;
    QCheckBox* chk_layer_swaths_ = nullptr;
    QCheckBox* chk_layer_path_ = nullptr;
    QCheckBox* chk_layer_trail_ = nullptr;
    QCheckBox* chk_layer_robot_ = nullptr;
    
    // Quick actions bar
    QWidget* quick_actions_bar_ = nullptr;
    QPushButton* btn_quick_generate_ = nullptr;
    QPushButton* btn_quick_publish_ = nullptr;
    QPushButton* btn_quick_start_ = nullptr;
    QLabel* lbl_quick_status_ = nullptr;
    
    // Collapsible sections (QToolBox)
    QToolBox* toolbox_ = nullptr;
    
    // Video streaming panel
    QDockWidget* video_dock_ = nullptr;
    VideoStreamWidget* video_widget_ = nullptr;
    QPushButton* btn_view_fov_ = nullptr;
    QRadioButton* radio_cam_left_ = nullptr;
    QRadioButton* radio_cam_right_ = nullptr;
    QSpinBox* spin_video_port_ = nullptr;
    QPushButton* btn_video_play_ = nullptr;
    QPushButton* btn_video_stop_ = nullptr;
    QLabel* lbl_video_status_ = nullptr;
    
    // ROS2 camera selection publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_select_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_status_sub_;
    QString current_streaming_camera_ = "left";
};

} // namespace f2c_cpp

