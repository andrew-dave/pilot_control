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
    QByteArrayData data[110];
    char stringdata0[1777];
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
QT_MOC_LITERAL(7, 120, 19), // "onRobotLoginClicked"
QT_MOC_LITERAL(8, 140, 16), // "onRobotIdChanged"
QT_MOC_LITERAL(9, 157, 7), // "robotId"
QT_MOC_LITERAL(10, 165, 20), // "onLoginCountdownTick"
QT_MOC_LITERAL(11, 186, 15), // "applyHeightCrop"
QT_MOC_LITERAL(12, 202, 15), // "applyDownsample"
QT_MOC_LITERAL(13, 218, 11), // "computeHull"
QT_MOC_LITERAL(14, 230, 15), // "simplifyPolygon"
QT_MOC_LITERAL(15, 246, 16), // "showPointCloud3D"
QT_MOC_LITERAL(16, 263, 18), // "toggleROISelection"
QT_MOC_LITERAL(17, 282, 8), // "clearROI"
QT_MOC_LITERAL(18, 291, 23), // "toggleObstacleSelection"
QT_MOC_LITERAL(19, 315, 19), // "autoDetectObstacles"
QT_MOC_LITERAL(20, 335, 22), // "deleteSelectedObstacle"
QT_MOC_LITERAL(21, 358, 14), // "clearObstacles"
QT_MOC_LITERAL(22, 373, 18), // "undoSelectionPoint"
QT_MOC_LITERAL(23, 392, 15), // "finishSelection"
QT_MOC_LITERAL(24, 408, 10), // "buildField"
QT_MOC_LITERAL(25, 419, 14), // "generateSwaths"
QT_MOC_LITERAL(26, 434, 13), // "generateRoute"
QT_MOC_LITERAL(27, 448, 12), // "generatePath"
QT_MOC_LITERAL(28, 461, 13), // "clearCoverage"
QT_MOC_LITERAL(29, 475, 13), // "exportPathCSV"
QT_MOC_LITERAL(30, 489, 16), // "publishWaypoints"
QT_MOC_LITERAL(31, 506, 15), // "startNavigation"
QT_MOC_LITERAL(32, 522, 12), // "planHomePath"
QT_MOC_LITERAL(33, 535, 15), // "clearRobotTrail"
QT_MOC_LITERAL(34, 551, 17), // "onPathModeChanged"
QT_MOC_LITERAL(35, 569, 13), // "onROISelected"
QT_MOC_LITERAL(36, 583, 9), // "Polygon2D"
QT_MOC_LITERAL(37, 593, 3), // "roi"
QT_MOC_LITERAL(38, 597, 18), // "onObstacleSelected"
QT_MOC_LITERAL(39, 616, 8), // "obstacle"
QT_MOC_LITERAL(40, 625, 20), // "onSelectionCancelled"
QT_MOC_LITERAL(41, 646, 17), // "toggleMeasureMode"
QT_MOC_LITERAL(42, 664, 24), // "onMeasureDistanceUpdated"
QT_MOC_LITERAL(43, 689, 10), // "distance_m"
QT_MOC_LITERAL(44, 700, 5), // "valid"
QT_MOC_LITERAL(45, 706, 25), // "onObstacleDeleteRequested"
QT_MOC_LITERAL(46, 732, 5), // "index"
QT_MOC_LITERAL(47, 738, 26), // "onObstacleSelectionChanged"
QT_MOC_LITERAL(48, 765, 29), // "onAutoDetectObstaclesFinished"
QT_MOC_LITERAL(49, 795, 18), // "updateDownsampleUI"
QT_MOC_LITERAL(50, 814, 6), // "method"
QT_MOC_LITERAL(51, 821, 16), // "tryReconnectROS2"
QT_MOC_LITERAL(52, 838, 22), // "checkZenohBridgeStatus"
QT_MOC_LITERAL(53, 861, 24), // "computeReprojectionError"
QT_MOC_LITERAL(54, 886, 22), // "clearReprojectionError"
QT_MOC_LITERAL(55, 909, 19), // "toggleRectangleMode"
QT_MOC_LITERAL(56, 929, 20), // "onRectangleCompleted"
QT_MOC_LITERAL(57, 950, 4), // "rect"
QT_MOC_LITERAL(58, 955, 14), // "toggleDarkMode"
QT_MOC_LITERAL(59, 970, 10), // "applyTheme"
QT_MOC_LITERAL(60, 981, 19), // "updateCoverageStats"
QT_MOC_LITERAL(61, 1001, 12), // "computeStats"
QT_MOC_LITERAL(62, 1014, 13), // "CoverageStats"
QT_MOC_LITERAL(63, 1028, 27), // "computeObstacleAvoidingPath"
QT_MOC_LITERAL(64, 1056, 13), // "PathStateList"
QT_MOC_LITERAL(65, 1070, 7), // "Point2D"
QT_MOC_LITERAL(66, 1078, 5), // "start"
QT_MOC_LITERAL(67, 1084, 4), // "goal"
QT_MOC_LITERAL(68, 1089, 16), // "spacing_override"
QT_MOC_LITERAL(69, 1106, 18), // "clearance_override"
QT_MOC_LITERAL(70, 1125, 19), // "updateWorkflowSteps"
QT_MOC_LITERAL(71, 1145, 21), // "onWorkflowStepClicked"
QT_MOC_LITERAL(72, 1167, 4), // "step"
QT_MOC_LITERAL(73, 1172, 21), // "updateLayerVisibility"
QT_MOC_LITERAL(74, 1194, 21), // "buildVideoPanelWidget"
QT_MOC_LITERAL(75, 1216, 8), // "QWidget*"
QT_MOC_LITERAL(76, 1225, 16), // "toggleVideoPanel"
QT_MOC_LITERAL(77, 1242, 15), // "onCameraToggled"
QT_MOC_LITERAL(78, 1258, 14), // "right_selected"
QT_MOC_LITERAL(79, 1273, 15), // "playVideoStream"
QT_MOC_LITERAL(80, 1289, 15), // "stopVideoStream"
QT_MOC_LITERAL(81, 1305, 22), // "onCameraStatusReceived"
QT_MOC_LITERAL(82, 1328, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(83, 1361, 3), // "msg"
QT_MOC_LITERAL(84, 1365, 22), // "openDataTransferDialog"
QT_MOC_LITERAL(85, 1388, 16), // "onTransferActive"
QT_MOC_LITERAL(86, 1405, 6), // "active"
QT_MOC_LITERAL(87, 1412, 18), // "onTransferProgress"
QT_MOC_LITERAL(88, 1431, 7), // "percent"
QT_MOC_LITERAL(89, 1439, 9), // "speedMBps"
QT_MOC_LITERAL(90, 1449, 29), // "onShowTransferDialogRequested"
QT_MOC_LITERAL(91, 1479, 25), // "onCancelTransferRequested"
QT_MOC_LITERAL(92, 1505, 16), // "onPresetSelected"
QT_MOC_LITERAL(93, 1522, 17), // "saveCurrentPreset"
QT_MOC_LITERAL(94, 1540, 15), // "createNewPreset"
QT_MOC_LITERAL(95, 1556, 17), // "openPresetManager"
QT_MOC_LITERAL(96, 1574, 10), // "loadPreset"
QT_MOC_LITERAL(97, 1585, 4), // "name"
QT_MOC_LITERAL(98, 1590, 17), // "refreshPresetList"
QT_MOC_LITERAL(99, 1608, 21), // "gatherCurrentSettings"
QT_MOC_LITERAL(100, 1630, 14), // "PlanningPreset"
QT_MOC_LITERAL(101, 1645, 11), // "applyPreset"
QT_MOC_LITERAL(102, 1657, 6), // "preset"
QT_MOC_LITERAL(103, 1664, 18), // "toggleTeleopWidget"
QT_MOC_LITERAL(104, 1683, 21), // "onTeleopStatusMessage"
QT_MOC_LITERAL(105, 1705, 7), // "message"
QT_MOC_LITERAL(106, 1713, 19), // "onCloudUploadActive"
QT_MOC_LITERAL(107, 1733, 16), // "startScanSession"
QT_MOC_LITERAL(108, 1750, 11), // "sectionName"
QT_MOC_LITERAL(109, 1762, 14) // "endScanSession"

    },
    "f2c_cpp::CoverageGUI\0loadPointCloud\0"
    "\0fetchLatestMapFromRobot\0"
    "loadPointCloudFromPath\0path\0"
    "alignLoadedMapToLatestRobotMap\0"
    "onRobotLoginClicked\0onRobotIdChanged\0"
    "robotId\0onLoginCountdownTick\0"
    "applyHeightCrop\0applyDownsample\0"
    "computeHull\0simplifyPolygon\0"
    "showPointCloud3D\0toggleROISelection\0"
    "clearROI\0toggleObstacleSelection\0"
    "autoDetectObstacles\0deleteSelectedObstacle\0"
    "clearObstacles\0undoSelectionPoint\0"
    "finishSelection\0buildField\0generateSwaths\0"
    "generateRoute\0generatePath\0clearCoverage\0"
    "exportPathCSV\0publishWaypoints\0"
    "startNavigation\0planHomePath\0"
    "clearRobotTrail\0onPathModeChanged\0"
    "onROISelected\0Polygon2D\0roi\0"
    "onObstacleSelected\0obstacle\0"
    "onSelectionCancelled\0toggleMeasureMode\0"
    "onMeasureDistanceUpdated\0distance_m\0"
    "valid\0onObstacleDeleteRequested\0index\0"
    "onObstacleSelectionChanged\0"
    "onAutoDetectObstaclesFinished\0"
    "updateDownsampleUI\0method\0tryReconnectROS2\0"
    "checkZenohBridgeStatus\0computeReprojectionError\0"
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
    "onCancelTransferRequested\0onPresetSelected\0"
    "saveCurrentPreset\0createNewPreset\0"
    "openPresetManager\0loadPreset\0name\0"
    "refreshPresetList\0gatherCurrentSettings\0"
    "PlanningPreset\0applyPreset\0preset\0"
    "toggleTeleopWidget\0onTeleopStatusMessage\0"
    "message\0onCloudUploadActive\0"
    "startScanSession\0sectionName\0"
    "endScanSession"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__CoverageGUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      80,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  414,    2, 0x08 /* Private */,
       3,    0,  415,    2, 0x08 /* Private */,
       4,    1,  416,    2, 0x08 /* Private */,
       6,    0,  419,    2, 0x08 /* Private */,
       7,    0,  420,    2, 0x08 /* Private */,
       8,    1,  421,    2, 0x08 /* Private */,
      10,    0,  424,    2, 0x08 /* Private */,
      11,    0,  425,    2, 0x08 /* Private */,
      12,    0,  426,    2, 0x08 /* Private */,
      13,    0,  427,    2, 0x08 /* Private */,
      14,    0,  428,    2, 0x08 /* Private */,
      15,    0,  429,    2, 0x08 /* Private */,
      16,    0,  430,    2, 0x08 /* Private */,
      17,    0,  431,    2, 0x08 /* Private */,
      18,    0,  432,    2, 0x08 /* Private */,
      19,    0,  433,    2, 0x08 /* Private */,
      20,    0,  434,    2, 0x08 /* Private */,
      21,    0,  435,    2, 0x08 /* Private */,
      22,    0,  436,    2, 0x08 /* Private */,
      23,    0,  437,    2, 0x08 /* Private */,
      24,    0,  438,    2, 0x08 /* Private */,
      25,    0,  439,    2, 0x08 /* Private */,
      26,    0,  440,    2, 0x08 /* Private */,
      27,    0,  441,    2, 0x08 /* Private */,
      28,    0,  442,    2, 0x08 /* Private */,
      29,    0,  443,    2, 0x08 /* Private */,
      30,    0,  444,    2, 0x08 /* Private */,
      31,    0,  445,    2, 0x08 /* Private */,
      32,    0,  446,    2, 0x08 /* Private */,
      33,    0,  447,    2, 0x08 /* Private */,
      34,    0,  448,    2, 0x08 /* Private */,
      35,    1,  449,    2, 0x08 /* Private */,
      38,    1,  452,    2, 0x08 /* Private */,
      40,    0,  455,    2, 0x08 /* Private */,
      41,    0,  456,    2, 0x08 /* Private */,
      42,    2,  457,    2, 0x08 /* Private */,
      45,    1,  462,    2, 0x08 /* Private */,
      47,    1,  465,    2, 0x08 /* Private */,
      48,    0,  468,    2, 0x08 /* Private */,
      49,    1,  469,    2, 0x08 /* Private */,
      51,    0,  472,    2, 0x08 /* Private */,
      52,    0,  473,    2, 0x08 /* Private */,
      53,    0,  474,    2, 0x08 /* Private */,
      54,    0,  475,    2, 0x08 /* Private */,
      55,    0,  476,    2, 0x08 /* Private */,
      56,    1,  477,    2, 0x08 /* Private */,
      58,    0,  480,    2, 0x08 /* Private */,
      59,    0,  481,    2, 0x08 /* Private */,
      60,    0,  482,    2, 0x08 /* Private */,
      61,    0,  483,    2, 0x08 /* Private */,
      63,    4,  484,    2, 0x08 /* Private */,
      63,    3,  493,    2, 0x28 /* Private | MethodCloned */,
      63,    2,  500,    2, 0x28 /* Private | MethodCloned */,
      70,    0,  505,    2, 0x08 /* Private */,
      71,    1,  506,    2, 0x08 /* Private */,
      73,    0,  509,    2, 0x08 /* Private */,
      74,    0,  510,    2, 0x08 /* Private */,
      76,    0,  511,    2, 0x08 /* Private */,
      77,    1,  512,    2, 0x08 /* Private */,
      79,    0,  515,    2, 0x08 /* Private */,
      80,    0,  516,    2, 0x08 /* Private */,
      81,    1,  517,    2, 0x08 /* Private */,
      84,    0,  520,    2, 0x08 /* Private */,
      85,    1,  521,    2, 0x08 /* Private */,
      87,    2,  524,    2, 0x08 /* Private */,
      90,    0,  529,    2, 0x08 /* Private */,
      91,    0,  530,    2, 0x08 /* Private */,
      92,    1,  531,    2, 0x08 /* Private */,
      93,    0,  534,    2, 0x08 /* Private */,
      94,    0,  535,    2, 0x08 /* Private */,
      95,    0,  536,    2, 0x08 /* Private */,
      96,    1,  537,    2, 0x08 /* Private */,
      98,    0,  540,    2, 0x08 /* Private */,
      99,    0,  541,    2, 0x08 /* Private */,
     101,    1,  542,    2, 0x08 /* Private */,
     103,    0,  545,    2, 0x08 /* Private */,
     104,    1,  546,    2, 0x08 /* Private */,
     106,    1,  549,    2, 0x08 /* Private */,
     107,    1,  552,    2, 0x08 /* Private */,
     109,    0,  555,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    9,
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
    QMetaType::Void, 0x80000000 | 36,   37,
    QMetaType::Void, 0x80000000 | 36,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Bool,   43,   44,
    QMetaType::Void, QMetaType::Int,   46,
    QMetaType::Void, QMetaType::Int,   46,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   50,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 36,   57,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 62,
    0x80000000 | 64, 0x80000000 | 65, 0x80000000 | 65, QMetaType::Double, QMetaType::Double,   66,   67,   68,   69,
    0x80000000 | 64, 0x80000000 | 65, 0x80000000 | 65, QMetaType::Double,   66,   67,   68,
    0x80000000 | 64, 0x80000000 | 65, 0x80000000 | 65,   66,   67,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   72,
    QMetaType::Void,
    0x80000000 | 75,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   78,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 82,   83,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   86,
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   88,   89,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   46,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   97,
    QMetaType::Void,
    0x80000000 | 100,
    QMetaType::Void, 0x80000000 | 100,  102,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  105,
    QMetaType::Void, QMetaType::Bool,   86,
    QMetaType::Void, QMetaType::QString,  108,
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
        case 4: _t->onRobotLoginClicked(); break;
        case 5: _t->onRobotIdChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 6: _t->onLoginCountdownTick(); break;
        case 7: _t->applyHeightCrop(); break;
        case 8: _t->applyDownsample(); break;
        case 9: _t->computeHull(); break;
        case 10: _t->simplifyPolygon(); break;
        case 11: _t->showPointCloud3D(); break;
        case 12: _t->toggleROISelection(); break;
        case 13: _t->clearROI(); break;
        case 14: _t->toggleObstacleSelection(); break;
        case 15: _t->autoDetectObstacles(); break;
        case 16: _t->deleteSelectedObstacle(); break;
        case 17: _t->clearObstacles(); break;
        case 18: _t->undoSelectionPoint(); break;
        case 19: _t->finishSelection(); break;
        case 20: _t->buildField(); break;
        case 21: _t->generateSwaths(); break;
        case 22: _t->generateRoute(); break;
        case 23: _t->generatePath(); break;
        case 24: _t->clearCoverage(); break;
        case 25: _t->exportPathCSV(); break;
        case 26: _t->publishWaypoints(); break;
        case 27: _t->startNavigation(); break;
        case 28: _t->planHomePath(); break;
        case 29: _t->clearRobotTrail(); break;
        case 30: _t->onPathModeChanged(); break;
        case 31: _t->onROISelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 32: _t->onObstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 33: _t->onSelectionCancelled(); break;
        case 34: _t->toggleMeasureMode(); break;
        case 35: _t->onMeasureDistanceUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 36: _t->onObstacleDeleteRequested((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 37: _t->onObstacleSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 38: _t->onAutoDetectObstaclesFinished(); break;
        case 39: _t->updateDownsampleUI((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 40: _t->tryReconnectROS2(); break;
        case 41: _t->checkZenohBridgeStatus(); break;
        case 42: _t->computeReprojectionError(); break;
        case 43: _t->clearReprojectionError(); break;
        case 44: _t->toggleRectangleMode(); break;
        case 45: _t->onRectangleCompleted((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 46: _t->toggleDarkMode(); break;
        case 47: _t->applyTheme(); break;
        case 48: _t->updateCoverageStats(); break;
        case 49: { CoverageStats _r = _t->computeStats();
            if (_a[0]) *reinterpret_cast< CoverageStats*>(_a[0]) = std::move(_r); }  break;
        case 50: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 51: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 52: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 53: _t->updateWorkflowSteps(); break;
        case 54: _t->onWorkflowStepClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 55: _t->updateLayerVisibility(); break;
        case 56: { QWidget* _r = _t->buildVideoPanelWidget();
            if (_a[0]) *reinterpret_cast< QWidget**>(_a[0]) = std::move(_r); }  break;
        case 57: _t->toggleVideoPanel(); break;
        case 58: _t->onCameraToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 59: _t->playVideoStream(); break;
        case 60: _t->stopVideoStream(); break;
        case 61: _t->onCameraStatusReceived((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 62: _t->openDataTransferDialog(); break;
        case 63: _t->onTransferActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 64: _t->onTransferProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 65: _t->onShowTransferDialogRequested(); break;
        case 66: _t->onCancelTransferRequested(); break;
        case 67: _t->onPresetSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 68: _t->saveCurrentPreset(); break;
        case 69: _t->createNewPreset(); break;
        case 70: _t->openPresetManager(); break;
        case 71: _t->loadPreset((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 72: _t->refreshPresetList(); break;
        case 73: { PlanningPreset _r = _t->gatherCurrentSettings();
            if (_a[0]) *reinterpret_cast< PlanningPreset*>(_a[0]) = std::move(_r); }  break;
        case 74: _t->applyPreset((*reinterpret_cast< const PlanningPreset(*)>(_a[1]))); break;
        case 75: _t->toggleTeleopWidget(); break;
        case 76: _t->onTeleopStatusMessage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 77: _t->onCloudUploadActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 78: _t->startScanSession((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 79: _t->endScanSession(); break;
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
        if (_id < 80)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 80;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 80)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 80;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
