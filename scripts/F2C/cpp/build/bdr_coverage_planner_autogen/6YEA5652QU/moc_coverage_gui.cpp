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
    QByteArrayData data[122];
    char stringdata0[2041];
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
QT_MOC_LITERAL(20, 335, 26), // "toggleObstacleRegionDelete"
QT_MOC_LITERAL(21, 362, 23), // "deleteObstaclesInRegion"
QT_MOC_LITERAL(22, 386, 9), // "Polygon2D"
QT_MOC_LITERAL(23, 396, 6), // "region"
QT_MOC_LITERAL(24, 403, 19), // "autoDetectObstacles"
QT_MOC_LITERAL(25, 423, 22), // "deleteSelectedObstacle"
QT_MOC_LITERAL(26, 446, 14), // "clearObstacles"
QT_MOC_LITERAL(27, 461, 18), // "undoSelectionPoint"
QT_MOC_LITERAL(28, 480, 15), // "finishSelection"
QT_MOC_LITERAL(29, 496, 10), // "buildField"
QT_MOC_LITERAL(30, 507, 14), // "generateSwaths"
QT_MOC_LITERAL(31, 522, 13), // "generateRoute"
QT_MOC_LITERAL(32, 536, 12), // "generatePath"
QT_MOC_LITERAL(33, 549, 13), // "clearCoverage"
QT_MOC_LITERAL(34, 563, 13), // "exportPathCSV"
QT_MOC_LITERAL(35, 577, 31), // "exportObstacleColoredPointCloud"
QT_MOC_LITERAL(36, 609, 16), // "publishWaypoints"
QT_MOC_LITERAL(37, 626, 15), // "startNavigation"
QT_MOC_LITERAL(38, 642, 12), // "planHomePath"
QT_MOC_LITERAL(39, 655, 13), // "onGoToClicked"
QT_MOC_LITERAL(40, 669, 15), // "clearRobotTrail"
QT_MOC_LITERAL(41, 685, 17), // "onPathModeChanged"
QT_MOC_LITERAL(42, 703, 13), // "onROISelected"
QT_MOC_LITERAL(43, 717, 3), // "roi"
QT_MOC_LITERAL(44, 721, 18), // "onObstacleSelected"
QT_MOC_LITERAL(45, 740, 8), // "obstacle"
QT_MOC_LITERAL(46, 749, 20), // "onSelectionCancelled"
QT_MOC_LITERAL(47, 770, 17), // "toggleMeasureMode"
QT_MOC_LITERAL(48, 788, 24), // "onMeasureDistanceUpdated"
QT_MOC_LITERAL(49, 813, 10), // "distance_m"
QT_MOC_LITERAL(50, 824, 5), // "valid"
QT_MOC_LITERAL(51, 830, 25), // "onObstacleDeleteRequested"
QT_MOC_LITERAL(52, 856, 5), // "index"
QT_MOC_LITERAL(53, 862, 26), // "onObstacleSelectionChanged"
QT_MOC_LITERAL(54, 889, 29), // "onAutoDetectObstaclesFinished"
QT_MOC_LITERAL(55, 919, 18), // "updateDownsampleUI"
QT_MOC_LITERAL(56, 938, 6), // "method"
QT_MOC_LITERAL(57, 945, 20), // "onHeightCropFinished"
QT_MOC_LITERAL(58, 966, 29), // "onTransitPathPlanningFinished"
QT_MOC_LITERAL(59, 996, 16), // "tryReconnectROS2"
QT_MOC_LITERAL(60, 1013, 22), // "checkZenohBridgeStatus"
QT_MOC_LITERAL(61, 1036, 24), // "computeReprojectionError"
QT_MOC_LITERAL(62, 1061, 22), // "clearReprojectionError"
QT_MOC_LITERAL(63, 1084, 19), // "toggleRectangleMode"
QT_MOC_LITERAL(64, 1104, 20), // "onRectangleCompleted"
QT_MOC_LITERAL(65, 1125, 4), // "rect"
QT_MOC_LITERAL(66, 1130, 14), // "toggleDarkMode"
QT_MOC_LITERAL(67, 1145, 10), // "applyTheme"
QT_MOC_LITERAL(68, 1156, 19), // "updateCoverageStats"
QT_MOC_LITERAL(69, 1176, 12), // "computeStats"
QT_MOC_LITERAL(70, 1189, 13), // "CoverageStats"
QT_MOC_LITERAL(71, 1203, 27), // "computeObstacleAvoidingPath"
QT_MOC_LITERAL(72, 1231, 13), // "PathStateList"
QT_MOC_LITERAL(73, 1245, 7), // "Point2D"
QT_MOC_LITERAL(74, 1253, 5), // "start"
QT_MOC_LITERAL(75, 1259, 4), // "goal"
QT_MOC_LITERAL(76, 1264, 16), // "spacing_override"
QT_MOC_LITERAL(77, 1281, 18), // "clearance_override"
QT_MOC_LITERAL(78, 1300, 19), // "updateWorkflowSteps"
QT_MOC_LITERAL(79, 1320, 21), // "onWorkflowStepClicked"
QT_MOC_LITERAL(80, 1342, 4), // "step"
QT_MOC_LITERAL(81, 1347, 21), // "updateLayerVisibility"
QT_MOC_LITERAL(82, 1369, 21), // "buildVideoPanelWidget"
QT_MOC_LITERAL(83, 1391, 8), // "QWidget*"
QT_MOC_LITERAL(84, 1400, 16), // "toggleVideoPanel"
QT_MOC_LITERAL(85, 1417, 15), // "onCameraToggled"
QT_MOC_LITERAL(86, 1433, 14), // "right_selected"
QT_MOC_LITERAL(87, 1448, 15), // "playVideoStream"
QT_MOC_LITERAL(88, 1464, 15), // "stopVideoStream"
QT_MOC_LITERAL(89, 1480, 22), // "onCameraStatusReceived"
QT_MOC_LITERAL(90, 1503, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(91, 1536, 3), // "msg"
QT_MOC_LITERAL(92, 1540, 22), // "openDataTransferDialog"
QT_MOC_LITERAL(93, 1563, 16), // "onTransferActive"
QT_MOC_LITERAL(94, 1580, 6), // "active"
QT_MOC_LITERAL(95, 1587, 18), // "onTransferProgress"
QT_MOC_LITERAL(96, 1606, 7), // "percent"
QT_MOC_LITERAL(97, 1614, 9), // "speedMBps"
QT_MOC_LITERAL(98, 1624, 29), // "onShowTransferDialogRequested"
QT_MOC_LITERAL(99, 1654, 25), // "onCancelTransferRequested"
QT_MOC_LITERAL(100, 1680, 25), // "onCancelProgressRequested"
QT_MOC_LITERAL(101, 1706, 16), // "onPresetSelected"
QT_MOC_LITERAL(102, 1723, 17), // "saveCurrentPreset"
QT_MOC_LITERAL(103, 1741, 15), // "createNewPreset"
QT_MOC_LITERAL(104, 1757, 17), // "openPresetManager"
QT_MOC_LITERAL(105, 1775, 10), // "loadPreset"
QT_MOC_LITERAL(106, 1786, 4), // "name"
QT_MOC_LITERAL(107, 1791, 17), // "refreshPresetList"
QT_MOC_LITERAL(108, 1809, 21), // "gatherCurrentSettings"
QT_MOC_LITERAL(109, 1831, 14), // "PlanningPreset"
QT_MOC_LITERAL(110, 1846, 11), // "applyPreset"
QT_MOC_LITERAL(111, 1858, 6), // "preset"
QT_MOC_LITERAL(112, 1865, 18), // "toggleTeleopWidget"
QT_MOC_LITERAL(113, 1884, 21), // "onTeleopStatusMessage"
QT_MOC_LITERAL(114, 1906, 7), // "message"
QT_MOC_LITERAL(115, 1914, 19), // "onCloudUploadActive"
QT_MOC_LITERAL(116, 1934, 16), // "startScanSession"
QT_MOC_LITERAL(117, 1951, 11), // "sectionName"
QT_MOC_LITERAL(118, 1963, 14), // "endScanSession"
QT_MOC_LITERAL(119, 1978, 18), // "onDcPauseRequested"
QT_MOC_LITERAL(120, 1997, 19), // "onDcResumeRequested"
QT_MOC_LITERAL(121, 2017, 23) // "onDcCancelScanRequested"

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
    "toggleObstacleRegionDelete\0"
    "deleteObstaclesInRegion\0Polygon2D\0"
    "region\0autoDetectObstacles\0"
    "deleteSelectedObstacle\0clearObstacles\0"
    "undoSelectionPoint\0finishSelection\0"
    "buildField\0generateSwaths\0generateRoute\0"
    "generatePath\0clearCoverage\0exportPathCSV\0"
    "exportObstacleColoredPointCloud\0"
    "publishWaypoints\0startNavigation\0"
    "planHomePath\0onGoToClicked\0clearRobotTrail\0"
    "onPathModeChanged\0onROISelected\0roi\0"
    "onObstacleSelected\0obstacle\0"
    "onSelectionCancelled\0toggleMeasureMode\0"
    "onMeasureDistanceUpdated\0distance_m\0"
    "valid\0onObstacleDeleteRequested\0index\0"
    "onObstacleSelectionChanged\0"
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
      91,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  469,    2, 0x08 /* Private */,
       3,    0,  470,    2, 0x08 /* Private */,
       4,    1,  471,    2, 0x08 /* Private */,
       6,    0,  474,    2, 0x08 /* Private */,
       7,    0,  475,    2, 0x08 /* Private */,
       8,    0,  476,    2, 0x08 /* Private */,
       9,    1,  477,    2, 0x08 /* Private */,
      11,    0,  480,    2, 0x08 /* Private */,
      12,    0,  481,    2, 0x08 /* Private */,
      13,    0,  482,    2, 0x08 /* Private */,
      14,    0,  483,    2, 0x08 /* Private */,
      15,    0,  484,    2, 0x08 /* Private */,
      16,    0,  485,    2, 0x08 /* Private */,
      17,    0,  486,    2, 0x08 /* Private */,
      18,    0,  487,    2, 0x08 /* Private */,
      19,    0,  488,    2, 0x08 /* Private */,
      20,    0,  489,    2, 0x08 /* Private */,
      21,    1,  490,    2, 0x08 /* Private */,
      24,    0,  493,    2, 0x08 /* Private */,
      25,    0,  494,    2, 0x08 /* Private */,
      26,    0,  495,    2, 0x08 /* Private */,
      27,    0,  496,    2, 0x08 /* Private */,
      28,    0,  497,    2, 0x08 /* Private */,
      29,    0,  498,    2, 0x08 /* Private */,
      30,    0,  499,    2, 0x08 /* Private */,
      31,    0,  500,    2, 0x08 /* Private */,
      32,    0,  501,    2, 0x08 /* Private */,
      33,    0,  502,    2, 0x08 /* Private */,
      34,    0,  503,    2, 0x08 /* Private */,
      35,    0,  504,    2, 0x08 /* Private */,
      36,    0,  505,    2, 0x08 /* Private */,
      37,    0,  506,    2, 0x08 /* Private */,
      38,    0,  507,    2, 0x08 /* Private */,
      39,    0,  508,    2, 0x08 /* Private */,
      40,    0,  509,    2, 0x08 /* Private */,
      41,    0,  510,    2, 0x08 /* Private */,
      42,    1,  511,    2, 0x08 /* Private */,
      44,    1,  514,    2, 0x08 /* Private */,
      46,    0,  517,    2, 0x08 /* Private */,
      47,    0,  518,    2, 0x08 /* Private */,
      48,    2,  519,    2, 0x08 /* Private */,
      51,    1,  524,    2, 0x08 /* Private */,
      53,    1,  527,    2, 0x08 /* Private */,
      54,    0,  530,    2, 0x08 /* Private */,
      55,    1,  531,    2, 0x08 /* Private */,
      57,    0,  534,    2, 0x08 /* Private */,
      58,    0,  535,    2, 0x08 /* Private */,
      59,    0,  536,    2, 0x08 /* Private */,
      60,    0,  537,    2, 0x08 /* Private */,
      61,    0,  538,    2, 0x08 /* Private */,
      62,    0,  539,    2, 0x08 /* Private */,
      63,    0,  540,    2, 0x08 /* Private */,
      64,    1,  541,    2, 0x08 /* Private */,
      66,    0,  544,    2, 0x08 /* Private */,
      67,    0,  545,    2, 0x08 /* Private */,
      68,    0,  546,    2, 0x08 /* Private */,
      69,    0,  547,    2, 0x08 /* Private */,
      71,    4,  548,    2, 0x08 /* Private */,
      71,    3,  557,    2, 0x28 /* Private | MethodCloned */,
      71,    2,  564,    2, 0x28 /* Private | MethodCloned */,
      78,    0,  569,    2, 0x08 /* Private */,
      79,    1,  570,    2, 0x08 /* Private */,
      81,    0,  573,    2, 0x08 /* Private */,
      82,    0,  574,    2, 0x08 /* Private */,
      84,    0,  575,    2, 0x08 /* Private */,
      85,    1,  576,    2, 0x08 /* Private */,
      87,    0,  579,    2, 0x08 /* Private */,
      88,    0,  580,    2, 0x08 /* Private */,
      89,    1,  581,    2, 0x08 /* Private */,
      92,    0,  584,    2, 0x08 /* Private */,
      93,    1,  585,    2, 0x08 /* Private */,
      95,    2,  588,    2, 0x08 /* Private */,
      98,    0,  593,    2, 0x08 /* Private */,
      99,    0,  594,    2, 0x08 /* Private */,
     100,    0,  595,    2, 0x08 /* Private */,
     101,    1,  596,    2, 0x08 /* Private */,
     102,    0,  599,    2, 0x08 /* Private */,
     103,    0,  600,    2, 0x08 /* Private */,
     104,    0,  601,    2, 0x08 /* Private */,
     105,    1,  602,    2, 0x08 /* Private */,
     107,    0,  605,    2, 0x08 /* Private */,
     108,    0,  606,    2, 0x08 /* Private */,
     110,    1,  607,    2, 0x08 /* Private */,
     112,    0,  610,    2, 0x08 /* Private */,
     113,    1,  611,    2, 0x08 /* Private */,
     115,    1,  614,    2, 0x08 /* Private */,
     116,    1,  617,    2, 0x08 /* Private */,
     118,    0,  620,    2, 0x08 /* Private */,
     119,    0,  621,    2, 0x08 /* Private */,
     120,    0,  622,    2, 0x08 /* Private */,
     121,    0,  623,    2, 0x08 /* Private */,

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
    QMetaType::Void, 0x80000000 | 22,   23,
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
    QMetaType::Void, 0x80000000 | 22,   43,
    QMetaType::Void, 0x80000000 | 22,   45,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Bool,   49,   50,
    QMetaType::Void, QMetaType::Int,   52,
    QMetaType::Void, QMetaType::Int,   52,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   56,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 22,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 70,
    0x80000000 | 72, 0x80000000 | 73, 0x80000000 | 73, QMetaType::Double, QMetaType::Double,   74,   75,   76,   77,
    0x80000000 | 72, 0x80000000 | 73, 0x80000000 | 73, QMetaType::Double,   74,   75,   76,
    0x80000000 | 72, 0x80000000 | 73, 0x80000000 | 73,   74,   75,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   80,
    QMetaType::Void,
    0x80000000 | 83,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   86,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 90,   91,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   94,
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   96,   97,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   52,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  106,
    QMetaType::Void,
    0x80000000 | 109,
    QMetaType::Void, 0x80000000 | 109,  111,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  114,
    QMetaType::Void, QMetaType::Bool,   94,
    QMetaType::Void, QMetaType::QString,  117,
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
        case 16: _t->toggleObstacleRegionDelete(); break;
        case 17: _t->deleteObstaclesInRegion((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 18: _t->autoDetectObstacles(); break;
        case 19: _t->deleteSelectedObstacle(); break;
        case 20: _t->clearObstacles(); break;
        case 21: _t->undoSelectionPoint(); break;
        case 22: _t->finishSelection(); break;
        case 23: _t->buildField(); break;
        case 24: _t->generateSwaths(); break;
        case 25: _t->generateRoute(); break;
        case 26: _t->generatePath(); break;
        case 27: _t->clearCoverage(); break;
        case 28: _t->exportPathCSV(); break;
        case 29: _t->exportObstacleColoredPointCloud(); break;
        case 30: _t->publishWaypoints(); break;
        case 31: _t->startNavigation(); break;
        case 32: _t->planHomePath(); break;
        case 33: _t->onGoToClicked(); break;
        case 34: _t->clearRobotTrail(); break;
        case 35: _t->onPathModeChanged(); break;
        case 36: _t->onROISelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 37: _t->onObstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 38: _t->onSelectionCancelled(); break;
        case 39: _t->toggleMeasureMode(); break;
        case 40: _t->onMeasureDistanceUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 41: _t->onObstacleDeleteRequested((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 42: _t->onObstacleSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 43: _t->onAutoDetectObstaclesFinished(); break;
        case 44: _t->updateDownsampleUI((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 45: _t->onHeightCropFinished(); break;
        case 46: _t->onTransitPathPlanningFinished(); break;
        case 47: _t->tryReconnectROS2(); break;
        case 48: _t->checkZenohBridgeStatus(); break;
        case 49: _t->computeReprojectionError(); break;
        case 50: _t->clearReprojectionError(); break;
        case 51: _t->toggleRectangleMode(); break;
        case 52: _t->onRectangleCompleted((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 53: _t->toggleDarkMode(); break;
        case 54: _t->applyTheme(); break;
        case 55: _t->updateCoverageStats(); break;
        case 56: { CoverageStats _r = _t->computeStats();
            if (_a[0]) *reinterpret_cast< CoverageStats*>(_a[0]) = std::move(_r); }  break;
        case 57: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 58: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 59: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 60: _t->updateWorkflowSteps(); break;
        case 61: _t->onWorkflowStepClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 62: _t->updateLayerVisibility(); break;
        case 63: { QWidget* _r = _t->buildVideoPanelWidget();
            if (_a[0]) *reinterpret_cast< QWidget**>(_a[0]) = std::move(_r); }  break;
        case 64: _t->toggleVideoPanel(); break;
        case 65: _t->onCameraToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 66: _t->playVideoStream(); break;
        case 67: _t->stopVideoStream(); break;
        case 68: _t->onCameraStatusReceived((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 69: _t->openDataTransferDialog(); break;
        case 70: _t->onTransferActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 71: _t->onTransferProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 72: _t->onShowTransferDialogRequested(); break;
        case 73: _t->onCancelTransferRequested(); break;
        case 74: _t->onCancelProgressRequested(); break;
        case 75: _t->onPresetSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 76: _t->saveCurrentPreset(); break;
        case 77: _t->createNewPreset(); break;
        case 78: _t->openPresetManager(); break;
        case 79: _t->loadPreset((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 80: _t->refreshPresetList(); break;
        case 81: { PlanningPreset _r = _t->gatherCurrentSettings();
            if (_a[0]) *reinterpret_cast< PlanningPreset*>(_a[0]) = std::move(_r); }  break;
        case 82: _t->applyPreset((*reinterpret_cast< const PlanningPreset(*)>(_a[1]))); break;
        case 83: _t->toggleTeleopWidget(); break;
        case 84: _t->onTeleopStatusMessage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 85: _t->onCloudUploadActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 86: _t->startScanSession((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 87: _t->endScanSession(); break;
        case 88: _t->onDcPauseRequested(); break;
        case 89: _t->onDcResumeRequested(); break;
        case 90: _t->onDcCancelScanRequested(); break;
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
        if (_id < 91)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 91;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 91)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 91;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
