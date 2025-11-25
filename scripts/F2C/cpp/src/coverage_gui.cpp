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
#include <cmath>
#include <algorithm>

namespace f2c_cpp {

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

void PlotWidget::clearAll() {
    points_.clear();
    polygon_.clear();
    roi_.clear();
    obstacles_.clear();
    swaths_.clear();
    route_.clear();
    path_.clear();
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
    
    setupUI();
    setupConnections();
    
    setStatus("Ready");
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
    
    controls_layout->addWidget(buildFileControls());
    controls_layout->addWidget(buildHeightControls());
    controls_layout->addWidget(buildDownsampleControls());
    controls_layout->addWidget(buildHullControls());
    controls_layout->addWidget(buildSimplifyControls());
    controls_layout->addWidget(buildCoverageControls());
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
    
    // Set progress callback
    setProgressCallback([this](int percent, const std::string& msg) {
        QMetaObject::invokeMethod(this, [this, percent, msg]() {
            updateProgress(percent, QString::fromStdString(msg));
        }, Qt::QueuedConnection);
    });
}

QGroupBox* CoverageGUI::buildFileControls() {
    QGroupBox* box = new QGroupBox("Point Cloud");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QPushButton* btn_load = new QPushButton("Load PCD / PLY / XYZ");
    btn_load->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
    connect(btn_load, &QPushButton::clicked, this, &CoverageGUI::loadPointCloud);
    v->addWidget(btn_load);
    
    lbl_file_ = new QLabel("No file loaded");
    v->addWidget(lbl_file_);
    
    return box;
}

QGroupBox* CoverageGUI::buildHeightControls() {
    QGroupBox* box = new QGroupBox("Height Cropping");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QHBoxLayout* h = new QHBoxLayout();
    h->addWidget(new QLabel("Z-band ±"));
    spin_z_band_ = new QDoubleSpinBox();
    spin_z_band_->setRange(0.0, 10.0);
    spin_z_band_->setSingleStep(0.05);
    spin_z_band_->setValue(0.5);
    h->addWidget(spin_z_band_);
    v->addLayout(h);
    
    QPushButton* btn_apply = new QPushButton("Apply Height Crop");
    btn_apply->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
    connect(btn_apply, &QPushButton::clicked, this, &CoverageGUI::applyHeightCrop);
    v->addWidget(btn_apply);
    
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

QGroupBox* CoverageGUI::buildCoverageControls() {
    QGroupBox* box = new QGroupBox("Fields2Cover & Coverage");
    QVBoxLayout* v = new QVBoxLayout(box);
    
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
    
    return box;
}

QGroupBox* CoverageGUI::buildExportControls() {
    QGroupBox* box = new QGroupBox("Export");
    QVBoxLayout* v = new QVBoxLayout(box);
    
    QPushButton* btn_export_path = new QPushButton("Export Path CSV");
    btn_export_path->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
    connect(btn_export_path, &QPushButton::clicked, this, &CoverageGUI::exportPathCSV);
    v->addWidget(btn_export_path);
    
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
    
    plot_->resetView();
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
        
        refreshPlot();
        setStatus(QString("Loaded %1 points").arg(pcd_points_->size()), 4000);
        
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
    
    showProgress(true, "Applying height crop...");
    
    try {
        filtered_points_ = filterByZBand(pcd_points_, spin_z_band_->value());
        
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
        
        refreshPlot();
        setStatus(QString("Filtered to %1 points").arg(filtered_points_->size()), 4000);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString::fromStdString(e.what()));
    }
    
    showProgress(false);
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
    if (path_.empty()) {
        QMessageBox::warning(this, "Warning", "Generate path first.");
        return;
    }
    
    QString filename = QFileDialog::getSaveFileName(this, "Save Path CSV", "", "CSV (*.csv)");
    if (filename.isEmpty()) return;
    
    if (!filename.toLower().endsWith(".csv")) {
        filename += ".csv";
    }
    
    if (savePathToCSV(path_, filename.toStdString())) {
        QMessageBox::information(this, "Export", 
                                QString("Saved %1 points").arg(path_.size()));
        setStatus("Path exported", 4000);
    } else {
        QMessageBox::critical(this, "Error", "Failed to save file");
    }
}

} // namespace f2c_cpp

