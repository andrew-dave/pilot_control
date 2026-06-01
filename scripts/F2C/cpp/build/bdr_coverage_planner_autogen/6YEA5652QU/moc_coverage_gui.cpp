/****************************************************************************
** Meta object code from reading C++ file 'coverage_gui.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/coverage_gui.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'coverage_gui.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_f2c_cpp__VideoStreamWidget_t {
    QByteArrayData data[7];
    char stringdata0[80];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_f2c_cpp__VideoStreamWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_f2c_cpp__VideoStreamWidget_t qt_meta_stringdata_f2c_cpp__VideoStreamWidget = {
    {
QT_MOC_LITERAL(0, 0, 26), // "f2c_cpp::VideoStreamWidget"
QT_MOC_LITERAL(1, 27, 11), // "streamError"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 3), // "msg"
QT_MOC_LITERAL(4, 44, 13), // "streamStarted"
QT_MOC_LITERAL(5, 58, 13), // "streamStopped"
QT_MOC_LITERAL(6, 72, 7) // "pollBus"

    },
    "f2c_cpp::VideoStreamWidget\0streamError\0"
    "\0msg\0streamStarted\0streamStopped\0"
    "pollBus"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__VideoStreamWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    0,   37,    2, 0x06 /* Public */,
       5,    0,   38,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   39,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void f2c_cpp::VideoStreamWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VideoStreamWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->streamError((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->streamStarted(); break;
        case 2: _t->streamStopped(); break;
        case 3: _t->pollBus(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VideoStreamWidget::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VideoStreamWidget::streamError)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (VideoStreamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VideoStreamWidget::streamStarted)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (VideoStreamWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VideoStreamWidget::streamStopped)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject f2c_cpp::VideoStreamWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_f2c_cpp__VideoStreamWidget.data,
    qt_meta_data_f2c_cpp__VideoStreamWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *f2c_cpp::VideoStreamWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *f2c_cpp::VideoStreamWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_f2c_cpp__VideoStreamWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int f2c_cpp::VideoStreamWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void f2c_cpp::VideoStreamWidget::streamError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void f2c_cpp::VideoStreamWidget::streamStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void f2c_cpp::VideoStreamWidget::streamStopped()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
struct qt_meta_stringdata_f2c_cpp__PlotWidget_t {
    QByteArrayData data[19];
    char stringdata0[249];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_f2c_cpp__PlotWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_f2c_cpp__PlotWidget_t qt_meta_stringdata_f2c_cpp__PlotWidget = {
    {
QT_MOC_LITERAL(0, 0, 19), // "f2c_cpp::PlotWidget"
QT_MOC_LITERAL(1, 20, 11), // "roiSelected"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 9), // "Polygon2D"
QT_MOC_LITERAL(4, 43, 3), // "roi"
QT_MOC_LITERAL(5, 47, 16), // "obstacleSelected"
QT_MOC_LITERAL(6, 64, 8), // "obstacle"
QT_MOC_LITERAL(7, 73, 18), // "selectionCancelled"
QT_MOC_LITERAL(8, 92, 24), // "obstacleSelectionChanged"
QT_MOC_LITERAL(9, 117, 5), // "index"
QT_MOC_LITERAL(10, 123, 23), // "obstacleDeleteRequested"
QT_MOC_LITERAL(11, 147, 23), // "customWaypointRequested"
QT_MOC_LITERAL(12, 171, 7), // "Point2D"
QT_MOC_LITERAL(13, 179, 5), // "point"
QT_MOC_LITERAL(14, 185, 18), // "rectangleCompleted"
QT_MOC_LITERAL(15, 204, 4), // "rect"
QT_MOC_LITERAL(16, 209, 22), // "measureDistanceUpdated"
QT_MOC_LITERAL(17, 232, 10), // "distance_m"
QT_MOC_LITERAL(18, 243, 5) // "valid"

    },
    "f2c_cpp::PlotWidget\0roiSelected\0\0"
    "Polygon2D\0roi\0obstacleSelected\0obstacle\0"
    "selectionCancelled\0obstacleSelectionChanged\0"
    "index\0obstacleDeleteRequested\0"
    "customWaypointRequested\0Point2D\0point\0"
    "rectangleCompleted\0rect\0measureDistanceUpdated\0"
    "distance_m\0valid"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__PlotWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       5,    1,   57,    2, 0x06 /* Public */,
       7,    0,   60,    2, 0x06 /* Public */,
       8,    1,   61,    2, 0x06 /* Public */,
      10,    1,   64,    2, 0x06 /* Public */,
      11,    1,   67,    2, 0x06 /* Public */,
      14,    1,   70,    2, 0x06 /* Public */,
      16,    2,   73,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void, 0x80000000 | 3,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::Bool,   17,   18,

       0        // eod
};

void f2c_cpp::PlotWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PlotWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->roiSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 1: _t->obstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 2: _t->selectionCancelled(); break;
        case 3: _t->obstacleSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->obstacleDeleteRequested((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->customWaypointRequested((*reinterpret_cast< const Point2D(*)>(_a[1]))); break;
        case 6: _t->rectangleCompleted((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 7: _t->measureDistanceUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PlotWidget::*)(const Polygon2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::roiSelected)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(const Polygon2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::obstacleSelected)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::selectionCancelled)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::obstacleSelectionChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::obstacleDeleteRequested)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(const Point2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::customWaypointRequested)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(const Polygon2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::rectangleCompleted)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (PlotWidget::*)(double , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::measureDistanceUpdated)) {
                *result = 7;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject f2c_cpp::PlotWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_f2c_cpp__PlotWidget.data,
    qt_meta_data_f2c_cpp__PlotWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *f2c_cpp::PlotWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *f2c_cpp::PlotWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_f2c_cpp__PlotWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int f2c_cpp::PlotWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void f2c_cpp::PlotWidget::roiSelected(const Polygon2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void f2c_cpp::PlotWidget::obstacleSelected(const Polygon2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void f2c_cpp::PlotWidget::selectionCancelled()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void f2c_cpp::PlotWidget::obstacleSelectionChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void f2c_cpp::PlotWidget::obstacleDeleteRequested(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void f2c_cpp::PlotWidget::customWaypointRequested(const Point2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void f2c_cpp::PlotWidget::rectangleCompleted(const Polygon2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void f2c_cpp::PlotWidget::measureDistanceUpdated(double _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}
struct qt_meta_stringdata_f2c_cpp__CoverageGUI_t {
    QByteArrayData data[119];
    char stringdata0[1983];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_f2c_cpp__CoverageGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_f2c_cpp__CoverageGUI_t qt_meta_stringdata_f2c_cpp__CoverageGUI = {
    {
QT_MOC_LITERAL(0, 0, 20), // "f2c_cpp::CoverageGUI"
QT_MOC_LITERAL(1, 21, 14), // "loadPointCloud"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 23), // "fetchLatestMapFromRobot"
QT_MOC_LITERAL(4, 61, 22), // "loadPointCloudFromPath"
QT_MOC_LITERAL(5, 84, 4), // "path"
QT_MOC_LITERAL(6, 89, 30), // "alignLoadedMapToLatestRobotMap"
QT_MOC_LITERAL(7, 120, 19), // "loadTrailFromRosbag"
QT_MOC_LITERAL(8, 140, 19), // "onRobotLoginClicked"
QT_MOC_LITERAL(9, 160, 16), // "onRobotIdChanged"
QT_MOC_LITERAL(10, 177, 7), // "robotId"
QT_MOC_LITERAL(11, 185, 20), // "onLoginCountdownTick"
QT_MOC_LITERAL(12, 206, 15), // "applyHeightCrop"
QT_MOC_LITERAL(13, 222, 15), // "applyDownsample"
QT_MOC_LITERAL(14, 238, 11), // "computeHull"
QT_MOC_LITERAL(15, 250, 15), // "simplifyPolygon"
QT_MOC_LITERAL(16, 266, 16), // "showPointCloud3D"
QT_MOC_LITERAL(17, 283, 18), // "toggleROISelection"
QT_MOC_LITERAL(18, 302, 8), // "clearROI"
QT_MOC_LITERAL(19, 311, 23), // "toggleObstacleSelection"
QT_MOC_LITERAL(20, 335, 19), // "autoDetectObstacles"
QT_MOC_LITERAL(21, 355, 22), // "deleteSelectedObstacle"
QT_MOC_LITERAL(22, 378, 14), // "clearObstacles"
QT_MOC_LITERAL(23, 393, 18), // "undoSelectionPoint"
QT_MOC_LITERAL(24, 412, 15), // "finishSelection"
QT_MOC_LITERAL(25, 428, 10), // "buildField"
QT_MOC_LITERAL(26, 439, 14), // "generateSwaths"
QT_MOC_LITERAL(27, 454, 13), // "generateRoute"
QT_MOC_LITERAL(28, 468, 12), // "generatePath"
QT_MOC_LITERAL(29, 481, 13), // "clearCoverage"
QT_MOC_LITERAL(30, 495, 13), // "exportPathCSV"
QT_MOC_LITERAL(31, 509, 31), // "exportObstacleColoredPointCloud"
QT_MOC_LITERAL(32, 541, 16), // "publishWaypoints"
QT_MOC_LITERAL(33, 558, 15), // "startNavigation"
QT_MOC_LITERAL(34, 574, 12), // "planHomePath"
QT_MOC_LITERAL(35, 587, 13), // "onGoToClicked"
QT_MOC_LITERAL(36, 601, 15), // "clearRobotTrail"
QT_MOC_LITERAL(37, 617, 17), // "onPathModeChanged"
QT_MOC_LITERAL(38, 635, 13), // "onROISelected"
QT_MOC_LITERAL(39, 649, 9), // "Polygon2D"
QT_MOC_LITERAL(40, 659, 3), // "roi"
QT_MOC_LITERAL(41, 663, 18), // "onObstacleSelected"
QT_MOC_LITERAL(42, 682, 8), // "obstacle"
QT_MOC_LITERAL(43, 691, 20), // "onSelectionCancelled"
QT_MOC_LITERAL(44, 712, 17), // "toggleMeasureMode"
QT_MOC_LITERAL(45, 730, 24), // "onMeasureDistanceUpdated"
QT_MOC_LITERAL(46, 755, 10), // "distance_m"
QT_MOC_LITERAL(47, 766, 5), // "valid"
QT_MOC_LITERAL(48, 772, 25), // "onObstacleDeleteRequested"
QT_MOC_LITERAL(49, 798, 5), // "index"
QT_MOC_LITERAL(50, 804, 26), // "onObstacleSelectionChanged"
QT_MOC_LITERAL(51, 831, 29), // "onAutoDetectObstaclesFinished"
QT_MOC_LITERAL(52, 861, 18), // "updateDownsampleUI"
QT_MOC_LITERAL(53, 880, 6), // "method"
QT_MOC_LITERAL(54, 887, 20), // "onHeightCropFinished"
QT_MOC_LITERAL(55, 908, 29), // "onTransitPathPlanningFinished"
QT_MOC_LITERAL(56, 938, 16), // "tryReconnectROS2"
QT_MOC_LITERAL(57, 955, 22), // "checkZenohBridgeStatus"
QT_MOC_LITERAL(58, 978, 24), // "computeReprojectionError"
QT_MOC_LITERAL(59, 1003, 22), // "clearReprojectionError"
QT_MOC_LITERAL(60, 1026, 19), // "toggleRectangleMode"
QT_MOC_LITERAL(61, 1046, 20), // "onRectangleCompleted"
QT_MOC_LITERAL(62, 1067, 4), // "rect"
QT_MOC_LITERAL(63, 1072, 14), // "toggleDarkMode"
QT_MOC_LITERAL(64, 1087, 10), // "applyTheme"
QT_MOC_LITERAL(65, 1098, 19), // "updateCoverageStats"
QT_MOC_LITERAL(66, 1118, 12), // "computeStats"
QT_MOC_LITERAL(67, 1131, 13), // "CoverageStats"
QT_MOC_LITERAL(68, 1145, 27), // "computeObstacleAvoidingPath"
QT_MOC_LITERAL(69, 1173, 13), // "PathStateList"
QT_MOC_LITERAL(70, 1187, 7), // "Point2D"
QT_MOC_LITERAL(71, 1195, 5), // "start"
QT_MOC_LITERAL(72, 1201, 4), // "goal"
QT_MOC_LITERAL(73, 1206, 16), // "spacing_override"
QT_MOC_LITERAL(74, 1223, 18), // "clearance_override"
QT_MOC_LITERAL(75, 1242, 19), // "updateWorkflowSteps"
QT_MOC_LITERAL(76, 1262, 21), // "onWorkflowStepClicked"
QT_MOC_LITERAL(77, 1284, 4), // "step"
QT_MOC_LITERAL(78, 1289, 21), // "updateLayerVisibility"
QT_MOC_LITERAL(79, 1311, 21), // "buildVideoPanelWidget"
QT_MOC_LITERAL(80, 1333, 8), // "QWidget*"
QT_MOC_LITERAL(81, 1342, 16), // "toggleVideoPanel"
QT_MOC_LITERAL(82, 1359, 15), // "onCameraToggled"
QT_MOC_LITERAL(83, 1375, 14), // "right_selected"
QT_MOC_LITERAL(84, 1390, 15), // "playVideoStream"
QT_MOC_LITERAL(85, 1406, 15), // "stopVideoStream"
QT_MOC_LITERAL(86, 1422, 22), // "onCameraStatusReceived"
QT_MOC_LITERAL(87, 1445, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(88, 1478, 3), // "msg"
QT_MOC_LITERAL(89, 1482, 22), // "openDataTransferDialog"
QT_MOC_LITERAL(90, 1505, 16), // "onTransferActive"
QT_MOC_LITERAL(91, 1522, 6), // "active"
QT_MOC_LITERAL(92, 1529, 18), // "onTransferProgress"
QT_MOC_LITERAL(93, 1548, 7), // "percent"
QT_MOC_LITERAL(94, 1556, 9), // "speedMBps"
QT_MOC_LITERAL(95, 1566, 29), // "onShowTransferDialogRequested"
QT_MOC_LITERAL(96, 1596, 25), // "onCancelTransferRequested"
QT_MOC_LITERAL(97, 1622, 25), // "onCancelProgressRequested"
QT_MOC_LITERAL(98, 1648, 16), // "onPresetSelected"
QT_MOC_LITERAL(99, 1665, 17), // "saveCurrentPreset"
QT_MOC_LITERAL(100, 1683, 15), // "createNewPreset"
QT_MOC_LITERAL(101, 1699, 17), // "openPresetManager"
QT_MOC_LITERAL(102, 1717, 10), // "loadPreset"
QT_MOC_LITERAL(103, 1728, 4), // "name"
QT_MOC_LITERAL(104, 1733, 17), // "refreshPresetList"
QT_MOC_LITERAL(105, 1751, 21), // "gatherCurrentSettings"
QT_MOC_LITERAL(106, 1773, 14), // "PlanningPreset"
QT_MOC_LITERAL(107, 1788, 11), // "applyPreset"
QT_MOC_LITERAL(108, 1800, 6), // "preset"
QT_MOC_LITERAL(109, 1807, 18), // "toggleTeleopWidget"
QT_MOC_LITERAL(110, 1826, 21), // "onTeleopStatusMessage"
QT_MOC_LITERAL(111, 1848, 7), // "message"
QT_MOC_LITERAL(112, 1856, 19), // "onCloudUploadActive"
QT_MOC_LITERAL(113, 1876, 16), // "startScanSession"
QT_MOC_LITERAL(114, 1893, 11), // "sectionName"
QT_MOC_LITERAL(115, 1905, 14), // "endScanSession"
QT_MOC_LITERAL(116, 1920, 18), // "onDcPauseRequested"
QT_MOC_LITERAL(117, 1939, 19), // "onDcResumeRequested"
QT_MOC_LITERAL(118, 1959, 23) // "onDcCancelScanRequested"

    },
    "f2c_cpp::CoverageGUI\0loadPointCloud\0"
    "\0fetchLatestMapFromRobot\0"
    "loadPointCloudFromPath\0path\0"
    "alignLoadedMapToLatestRobotMap\0"
    "loadTrailFromRosbag\0onRobotLoginClicked\0"
    "onRobotIdChanged\0robotId\0onLoginCountdownTick\0"
    "applyHeightCrop\0applyDownsample\0"
    "computeHull\0simplifyPolygon\0"
    "showPointCloud3D\0toggleROISelection\0"
    "clearROI\0toggleObstacleSelection\0"
    "autoDetectObstacles\0deleteSelectedObstacle\0"
    "clearObstacles\0undoSelectionPoint\0"
    "finishSelection\0buildField\0generateSwaths\0"
    "generateRoute\0generatePath\0clearCoverage\0"
    "exportPathCSV\0exportObstacleColoredPointCloud\0"
    "publishWaypoints\0startNavigation\0"
    "planHomePath\0onGoToClicked\0clearRobotTrail\0"
    "onPathModeChanged\0onROISelected\0"
    "Polygon2D\0roi\0onObstacleSelected\0"
    "obstacle\0onSelectionCancelled\0"
    "toggleMeasureMode\0onMeasureDistanceUpdated\0"
    "distance_m\0valid\0onObstacleDeleteRequested\0"
    "index\0onObstacleSelectionChanged\0"
    "onAutoDetectObstaclesFinished\0"
    "updateDownsampleUI\0method\0"
    "onHeightCropFinished\0onTransitPathPlanningFinished\0"
    "tryReconnectROS2\0checkZenohBridgeStatus\0"
    "computeReprojectionError\0"
    "clearReprojectionError\0toggleRectangleMode\0"
    "onRectangleCompleted\0rect\0toggleDarkMode\0"
    "applyTheme\0updateCoverageStats\0"
    "computeStats\0CoverageStats\0"
    "computeObstacleAvoidingPath\0PathStateList\0"
    "Point2D\0start\0goal\0spacing_override\0"
    "clearance_override\0updateWorkflowSteps\0"
    "onWorkflowStepClicked\0step\0"
    "updateLayerVisibility\0buildVideoPanelWidget\0"
    "QWidget*\0toggleVideoPanel\0onCameraToggled\0"
    "right_selected\0playVideoStream\0"
    "stopVideoStream\0onCameraStatusReceived\0"
    "std_msgs::msg::String::SharedPtr\0msg\0"
    "openDataTransferDialog\0onTransferActive\0"
    "active\0onTransferProgress\0percent\0"
    "speedMBps\0onShowTransferDialogRequested\0"
    "onCancelTransferRequested\0"
    "onCancelProgressRequested\0onPresetSelected\0"
    "saveCurrentPreset\0createNewPreset\0"
    "openPresetManager\0loadPreset\0name\0"
    "refreshPresetList\0gatherCurrentSettings\0"
    "PlanningPreset\0applyPreset\0preset\0"
    "toggleTeleopWidget\0onTeleopStatusMessage\0"
    "message\0onCloudUploadActive\0"
    "startScanSession\0sectionName\0"
    "endScanSession\0onDcPauseRequested\0"
    "onDcResumeRequested\0onDcCancelScanRequested"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__CoverageGUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      89,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  459,    2, 0x08 /* Private */,
       3,    0,  460,    2, 0x08 /* Private */,
       4,    1,  461,    2, 0x08 /* Private */,
       6,    0,  464,    2, 0x08 /* Private */,
       7,    0,  465,    2, 0x08 /* Private */,
       8,    0,  466,    2, 0x08 /* Private */,
       9,    1,  467,    2, 0x08 /* Private */,
      11,    0,  470,    2, 0x08 /* Private */,
      12,    0,  471,    2, 0x08 /* Private */,
      13,    0,  472,    2, 0x08 /* Private */,
      14,    0,  473,    2, 0x08 /* Private */,
      15,    0,  474,    2, 0x08 /* Private */,
      16,    0,  475,    2, 0x08 /* Private */,
      17,    0,  476,    2, 0x08 /* Private */,
      18,    0,  477,    2, 0x08 /* Private */,
      19,    0,  478,    2, 0x08 /* Private */,
      20,    0,  479,    2, 0x08 /* Private */,
      21,    0,  480,    2, 0x08 /* Private */,
      22,    0,  481,    2, 0x08 /* Private */,
      23,    0,  482,    2, 0x08 /* Private */,
      24,    0,  483,    2, 0x08 /* Private */,
      25,    0,  484,    2, 0x08 /* Private */,
      26,    0,  485,    2, 0x08 /* Private */,
      27,    0,  486,    2, 0x08 /* Private */,
      28,    0,  487,    2, 0x08 /* Private */,
      29,    0,  488,    2, 0x08 /* Private */,
      30,    0,  489,    2, 0x08 /* Private */,
      31,    0,  490,    2, 0x08 /* Private */,
      32,    0,  491,    2, 0x08 /* Private */,
      33,    0,  492,    2, 0x08 /* Private */,
      34,    0,  493,    2, 0x08 /* Private */,
      35,    0,  494,    2, 0x08 /* Private */,
      36,    0,  495,    2, 0x08 /* Private */,
      37,    0,  496,    2, 0x08 /* Private */,
      38,    1,  497,    2, 0x08 /* Private */,
      41,    1,  500,    2, 0x08 /* Private */,
      43,    0,  503,    2, 0x08 /* Private */,
      44,    0,  504,    2, 0x08 /* Private */,
      45,    2,  505,    2, 0x08 /* Private */,
      48,    1,  510,    2, 0x08 /* Private */,
      50,    1,  513,    2, 0x08 /* Private */,
      51,    0,  516,    2, 0x08 /* Private */,
      52,    1,  517,    2, 0x08 /* Private */,
      54,    0,  520,    2, 0x08 /* Private */,
      55,    0,  521,    2, 0x08 /* Private */,
      56,    0,  522,    2, 0x08 /* Private */,
      57,    0,  523,    2, 0x08 /* Private */,
      58,    0,  524,    2, 0x08 /* Private */,
      59,    0,  525,    2, 0x08 /* Private */,
      60,    0,  526,    2, 0x08 /* Private */,
      61,    1,  527,    2, 0x08 /* Private */,
      63,    0,  530,    2, 0x08 /* Private */,
      64,    0,  531,    2, 0x08 /* Private */,
      65,    0,  532,    2, 0x08 /* Private */,
      66,    0,  533,    2, 0x08 /* Private */,
      68,    4,  534,    2, 0x08 /* Private */,
      68,    3,  543,    2, 0x28 /* Private | MethodCloned */,
      68,    2,  550,    2, 0x28 /* Private | MethodCloned */,
      75,    0,  555,    2, 0x08 /* Private */,
      76,    1,  556,    2, 0x08 /* Private */,
      78,    0,  559,    2, 0x08 /* Private */,
      79,    0,  560,    2, 0x08 /* Private */,
      81,    0,  561,    2, 0x08 /* Private */,
      82,    1,  562,    2, 0x08 /* Private */,
      84,    0,  565,    2, 0x08 /* Private */,
      85,    0,  566,    2, 0x08 /* Private */,
      86,    1,  567,    2, 0x08 /* Private */,
      89,    0,  570,    2, 0x08 /* Private */,
      90,    1,  571,    2, 0x08 /* Private */,
      92,    2,  574,    2, 0x08 /* Private */,
      95,    0,  579,    2, 0x08 /* Private */,
      96,    0,  580,    2, 0x08 /* Private */,
      97,    0,  581,    2, 0x08 /* Private */,
      98,    1,  582,    2, 0x08 /* Private */,
      99,    0,  585,    2, 0x08 /* Private */,
     100,    0,  586,    2, 0x08 /* Private */,
     101,    0,  587,    2, 0x08 /* Private */,
     102,    1,  588,    2, 0x08 /* Private */,
     104,    0,  591,    2, 0x08 /* Private */,
     105,    0,  592,    2, 0x08 /* Private */,
     107,    1,  593,    2, 0x08 /* Private */,
     109,    0,  596,    2, 0x08 /* Private */,
     110,    1,  597,    2, 0x08 /* Private */,
     112,    1,  600,    2, 0x08 /* Private */,
     113,    1,  603,    2, 0x08 /* Private */,
     115,    0,  606,    2, 0x08 /* Private */,
     116,    0,  607,    2, 0x08 /* Private */,
     117,    0,  608,    2, 0x08 /* Private */,
     118,    0,  609,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 39,   40,
    QMetaType::Void, 0x80000000 | 39,   42,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Bool,   46,   47,
    QMetaType::Void, QMetaType::Int,   49,
    QMetaType::Void, QMetaType::Int,   49,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   53,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 39,   62,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 67,
    0x80000000 | 69, 0x80000000 | 70, 0x80000000 | 70, QMetaType::Double, QMetaType::Double,   71,   72,   73,   74,
    0x80000000 | 69, 0x80000000 | 70, 0x80000000 | 70, QMetaType::Double,   71,   72,   73,
    0x80000000 | 69, 0x80000000 | 70, 0x80000000 | 70,   71,   72,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void,
    0x80000000 | 80,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   83,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 87,   88,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   91,
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   93,   94,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   49,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  103,
    QMetaType::Void,
    0x80000000 | 106,
    QMetaType::Void, 0x80000000 | 106,  108,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  111,
    QMetaType::Void, QMetaType::Bool,   91,
    QMetaType::Void, QMetaType::QString,  114,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void f2c_cpp::CoverageGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CoverageGUI *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->loadPointCloud(); break;
        case 1: _t->fetchLatestMapFromRobot(); break;
        case 2: _t->loadPointCloudFromPath((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->alignLoadedMapToLatestRobotMap(); break;
        case 4: _t->loadTrailFromRosbag(); break;
        case 5: _t->onRobotLoginClicked(); break;
        case 6: _t->onRobotIdChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->onLoginCountdownTick(); break;
        case 8: _t->applyHeightCrop(); break;
        case 9: _t->applyDownsample(); break;
        case 10: _t->computeHull(); break;
        case 11: _t->simplifyPolygon(); break;
        case 12: _t->showPointCloud3D(); break;
        case 13: _t->toggleROISelection(); break;
        case 14: _t->clearROI(); break;
        case 15: _t->toggleObstacleSelection(); break;
        case 16: _t->autoDetectObstacles(); break;
        case 17: _t->deleteSelectedObstacle(); break;
        case 18: _t->clearObstacles(); break;
        case 19: _t->undoSelectionPoint(); break;
        case 20: _t->finishSelection(); break;
        case 21: _t->buildField(); break;
        case 22: _t->generateSwaths(); break;
        case 23: _t->generateRoute(); break;
        case 24: _t->generatePath(); break;
        case 25: _t->clearCoverage(); break;
        case 26: _t->exportPathCSV(); break;
        case 27: _t->exportObstacleColoredPointCloud(); break;
        case 28: _t->publishWaypoints(); break;
        case 29: _t->startNavigation(); break;
        case 30: _t->planHomePath(); break;
        case 31: _t->onGoToClicked(); break;
        case 32: _t->clearRobotTrail(); break;
        case 33: _t->onPathModeChanged(); break;
        case 34: _t->onROISelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 35: _t->onObstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 36: _t->onSelectionCancelled(); break;
        case 37: _t->toggleMeasureMode(); break;
        case 38: _t->onMeasureDistanceUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 39: _t->onObstacleDeleteRequested((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 40: _t->onObstacleSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 41: _t->onAutoDetectObstaclesFinished(); break;
        case 42: _t->updateDownsampleUI((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 43: _t->onHeightCropFinished(); break;
        case 44: _t->onTransitPathPlanningFinished(); break;
        case 45: _t->tryReconnectROS2(); break;
        case 46: _t->checkZenohBridgeStatus(); break;
        case 47: _t->computeReprojectionError(); break;
        case 48: _t->clearReprojectionError(); break;
        case 49: _t->toggleRectangleMode(); break;
        case 50: _t->onRectangleCompleted((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 51: _t->toggleDarkMode(); break;
        case 52: _t->applyTheme(); break;
        case 53: _t->updateCoverageStats(); break;
        case 54: { CoverageStats _r = _t->computeStats();
            if (_a[0]) *reinterpret_cast< CoverageStats*>(_a[0]) = std::move(_r); }  break;
        case 55: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 56: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 57: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 58: _t->updateWorkflowSteps(); break;
        case 59: _t->onWorkflowStepClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 60: _t->updateLayerVisibility(); break;
        case 61: { QWidget* _r = _t->buildVideoPanelWidget();
            if (_a[0]) *reinterpret_cast< QWidget**>(_a[0]) = std::move(_r); }  break;
        case 62: _t->toggleVideoPanel(); break;
        case 63: _t->onCameraToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 64: _t->playVideoStream(); break;
        case 65: _t->stopVideoStream(); break;
        case 66: _t->onCameraStatusReceived((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 67: _t->openDataTransferDialog(); break;
        case 68: _t->onTransferActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 69: _t->onTransferProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 70: _t->onShowTransferDialogRequested(); break;
        case 71: _t->onCancelTransferRequested(); break;
        case 72: _t->onCancelProgressRequested(); break;
        case 73: _t->onPresetSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 74: _t->saveCurrentPreset(); break;
        case 75: _t->createNewPreset(); break;
        case 76: _t->openPresetManager(); break;
        case 77: _t->loadPreset((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 78: _t->refreshPresetList(); break;
        case 79: { PlanningPreset _r = _t->gatherCurrentSettings();
            if (_a[0]) *reinterpret_cast< PlanningPreset*>(_a[0]) = std::move(_r); }  break;
        case 80: _t->applyPreset((*reinterpret_cast< const PlanningPreset(*)>(_a[1]))); break;
        case 81: _t->toggleTeleopWidget(); break;
        case 82: _t->onTeleopStatusMessage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 83: _t->onCloudUploadActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 84: _t->startScanSession((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 85: _t->endScanSession(); break;
        case 86: _t->onDcPauseRequested(); break;
        case 87: _t->onDcResumeRequested(); break;
        case 88: _t->onDcCancelScanRequested(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject f2c_cpp::CoverageGUI::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_f2c_cpp__CoverageGUI.data,
    qt_meta_data_f2c_cpp__CoverageGUI,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *f2c_cpp::CoverageGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *f2c_cpp::CoverageGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_f2c_cpp__CoverageGUI.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int f2c_cpp::CoverageGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 89)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 89;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 89)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 89;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
