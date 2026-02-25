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
    QByteArrayData data[16];
    char stringdata0[209];
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
QT_MOC_LITERAL(15, 204, 4) // "rect"

    },
    "f2c_cpp::PlotWidget\0roiSelected\0\0"
    "Polygon2D\0roi\0obstacleSelected\0obstacle\0"
    "selectionCancelled\0obstacleSelectionChanged\0"
    "index\0obstacleDeleteRequested\0"
    "customWaypointRequested\0Point2D\0point\0"
    "rectangleCompleted\0rect"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__PlotWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       5,    1,   52,    2, 0x06 /* Public */,
       7,    0,   55,    2, 0x06 /* Public */,
       8,    1,   56,    2, 0x06 /* Public */,
      10,    1,   59,    2, 0x06 /* Public */,
      11,    1,   62,    2, 0x06 /* Public */,
      14,    1,   65,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void, 0x80000000 | 3,   15,

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
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
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
struct qt_meta_stringdata_f2c_cpp__CoverageGUI_t {
    QByteArrayData data[105];
    char stringdata0[1686];
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
QT_MOC_LITERAL(6, 89, 19), // "onRobotLoginClicked"
QT_MOC_LITERAL(7, 109, 16), // "onRobotIdChanged"
QT_MOC_LITERAL(8, 126, 7), // "robotId"
QT_MOC_LITERAL(9, 134, 20), // "onLoginCountdownTick"
QT_MOC_LITERAL(10, 155, 15), // "applyHeightCrop"
QT_MOC_LITERAL(11, 171, 15), // "applyDownsample"
QT_MOC_LITERAL(12, 187, 11), // "computeHull"
QT_MOC_LITERAL(13, 199, 15), // "simplifyPolygon"
QT_MOC_LITERAL(14, 215, 16), // "showPointCloud3D"
QT_MOC_LITERAL(15, 232, 18), // "toggleROISelection"
QT_MOC_LITERAL(16, 251, 8), // "clearROI"
QT_MOC_LITERAL(17, 260, 23), // "toggleObstacleSelection"
QT_MOC_LITERAL(18, 284, 19), // "autoDetectObstacles"
QT_MOC_LITERAL(19, 304, 22), // "deleteSelectedObstacle"
QT_MOC_LITERAL(20, 327, 14), // "clearObstacles"
QT_MOC_LITERAL(21, 342, 18), // "undoSelectionPoint"
QT_MOC_LITERAL(22, 361, 15), // "finishSelection"
QT_MOC_LITERAL(23, 377, 10), // "buildField"
QT_MOC_LITERAL(24, 388, 14), // "generateSwaths"
QT_MOC_LITERAL(25, 403, 13), // "generateRoute"
QT_MOC_LITERAL(26, 417, 12), // "generatePath"
QT_MOC_LITERAL(27, 430, 13), // "clearCoverage"
QT_MOC_LITERAL(28, 444, 13), // "exportPathCSV"
QT_MOC_LITERAL(29, 458, 16), // "publishWaypoints"
QT_MOC_LITERAL(30, 475, 15), // "startNavigation"
QT_MOC_LITERAL(31, 491, 12), // "planHomePath"
QT_MOC_LITERAL(32, 504, 15), // "clearRobotTrail"
QT_MOC_LITERAL(33, 520, 17), // "onPathModeChanged"
QT_MOC_LITERAL(34, 538, 13), // "onROISelected"
QT_MOC_LITERAL(35, 552, 9), // "Polygon2D"
QT_MOC_LITERAL(36, 562, 3), // "roi"
QT_MOC_LITERAL(37, 566, 18), // "onObstacleSelected"
QT_MOC_LITERAL(38, 585, 8), // "obstacle"
QT_MOC_LITERAL(39, 594, 20), // "onSelectionCancelled"
QT_MOC_LITERAL(40, 615, 25), // "onObstacleDeleteRequested"
QT_MOC_LITERAL(41, 641, 5), // "index"
QT_MOC_LITERAL(42, 647, 26), // "onObstacleSelectionChanged"
QT_MOC_LITERAL(43, 674, 29), // "onAutoDetectObstaclesFinished"
QT_MOC_LITERAL(44, 704, 18), // "updateDownsampleUI"
QT_MOC_LITERAL(45, 723, 6), // "method"
QT_MOC_LITERAL(46, 730, 16), // "tryReconnectROS2"
QT_MOC_LITERAL(47, 747, 22), // "checkZenohBridgeStatus"
QT_MOC_LITERAL(48, 770, 24), // "computeReprojectionError"
QT_MOC_LITERAL(49, 795, 22), // "clearReprojectionError"
QT_MOC_LITERAL(50, 818, 19), // "toggleRectangleMode"
QT_MOC_LITERAL(51, 838, 20), // "onRectangleCompleted"
QT_MOC_LITERAL(52, 859, 4), // "rect"
QT_MOC_LITERAL(53, 864, 14), // "toggleDarkMode"
QT_MOC_LITERAL(54, 879, 10), // "applyTheme"
QT_MOC_LITERAL(55, 890, 19), // "updateCoverageStats"
QT_MOC_LITERAL(56, 910, 12), // "computeStats"
QT_MOC_LITERAL(57, 923, 13), // "CoverageStats"
QT_MOC_LITERAL(58, 937, 27), // "computeObstacleAvoidingPath"
QT_MOC_LITERAL(59, 965, 13), // "PathStateList"
QT_MOC_LITERAL(60, 979, 7), // "Point2D"
QT_MOC_LITERAL(61, 987, 5), // "start"
QT_MOC_LITERAL(62, 993, 4), // "goal"
QT_MOC_LITERAL(63, 998, 16), // "spacing_override"
QT_MOC_LITERAL(64, 1015, 18), // "clearance_override"
QT_MOC_LITERAL(65, 1034, 19), // "updateWorkflowSteps"
QT_MOC_LITERAL(66, 1054, 21), // "onWorkflowStepClicked"
QT_MOC_LITERAL(67, 1076, 4), // "step"
QT_MOC_LITERAL(68, 1081, 21), // "updateLayerVisibility"
QT_MOC_LITERAL(69, 1103, 21), // "buildVideoPanelWidget"
QT_MOC_LITERAL(70, 1125, 8), // "QWidget*"
QT_MOC_LITERAL(71, 1134, 16), // "toggleVideoPanel"
QT_MOC_LITERAL(72, 1151, 15), // "onCameraToggled"
QT_MOC_LITERAL(73, 1167, 14), // "right_selected"
QT_MOC_LITERAL(74, 1182, 15), // "playVideoStream"
QT_MOC_LITERAL(75, 1198, 15), // "stopVideoStream"
QT_MOC_LITERAL(76, 1214, 22), // "onCameraStatusReceived"
QT_MOC_LITERAL(77, 1237, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(78, 1270, 3), // "msg"
QT_MOC_LITERAL(79, 1274, 22), // "openDataTransferDialog"
QT_MOC_LITERAL(80, 1297, 16), // "onTransferActive"
QT_MOC_LITERAL(81, 1314, 6), // "active"
QT_MOC_LITERAL(82, 1321, 18), // "onTransferProgress"
QT_MOC_LITERAL(83, 1340, 7), // "percent"
QT_MOC_LITERAL(84, 1348, 9), // "speedMBps"
QT_MOC_LITERAL(85, 1358, 29), // "onShowTransferDialogRequested"
QT_MOC_LITERAL(86, 1388, 25), // "onCancelTransferRequested"
QT_MOC_LITERAL(87, 1414, 16), // "onPresetSelected"
QT_MOC_LITERAL(88, 1431, 17), // "saveCurrentPreset"
QT_MOC_LITERAL(89, 1449, 15), // "createNewPreset"
QT_MOC_LITERAL(90, 1465, 17), // "openPresetManager"
QT_MOC_LITERAL(91, 1483, 10), // "loadPreset"
QT_MOC_LITERAL(92, 1494, 4), // "name"
QT_MOC_LITERAL(93, 1499, 17), // "refreshPresetList"
QT_MOC_LITERAL(94, 1517, 21), // "gatherCurrentSettings"
QT_MOC_LITERAL(95, 1539, 14), // "PlanningPreset"
QT_MOC_LITERAL(96, 1554, 11), // "applyPreset"
QT_MOC_LITERAL(97, 1566, 6), // "preset"
QT_MOC_LITERAL(98, 1573, 18), // "toggleTeleopWidget"
QT_MOC_LITERAL(99, 1592, 21), // "onTeleopStatusMessage"
QT_MOC_LITERAL(100, 1614, 7), // "message"
QT_MOC_LITERAL(101, 1622, 19), // "onCloudUploadActive"
QT_MOC_LITERAL(102, 1642, 16), // "startScanSession"
QT_MOC_LITERAL(103, 1659, 11), // "sectionName"
QT_MOC_LITERAL(104, 1671, 14) // "endScanSession"

    },
    "f2c_cpp::CoverageGUI\0loadPointCloud\0"
    "\0fetchLatestMapFromRobot\0"
    "loadPointCloudFromPath\0path\0"
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
    "onSelectionCancelled\0onObstacleDeleteRequested\0"
    "index\0onObstacleSelectionChanged\0"
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
      77,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  399,    2, 0x08 /* Private */,
       3,    0,  400,    2, 0x08 /* Private */,
       4,    1,  401,    2, 0x08 /* Private */,
       6,    0,  404,    2, 0x08 /* Private */,
       7,    1,  405,    2, 0x08 /* Private */,
       9,    0,  408,    2, 0x08 /* Private */,
      10,    0,  409,    2, 0x08 /* Private */,
      11,    0,  410,    2, 0x08 /* Private */,
      12,    0,  411,    2, 0x08 /* Private */,
      13,    0,  412,    2, 0x08 /* Private */,
      14,    0,  413,    2, 0x08 /* Private */,
      15,    0,  414,    2, 0x08 /* Private */,
      16,    0,  415,    2, 0x08 /* Private */,
      17,    0,  416,    2, 0x08 /* Private */,
      18,    0,  417,    2, 0x08 /* Private */,
      19,    0,  418,    2, 0x08 /* Private */,
      20,    0,  419,    2, 0x08 /* Private */,
      21,    0,  420,    2, 0x08 /* Private */,
      22,    0,  421,    2, 0x08 /* Private */,
      23,    0,  422,    2, 0x08 /* Private */,
      24,    0,  423,    2, 0x08 /* Private */,
      25,    0,  424,    2, 0x08 /* Private */,
      26,    0,  425,    2, 0x08 /* Private */,
      27,    0,  426,    2, 0x08 /* Private */,
      28,    0,  427,    2, 0x08 /* Private */,
      29,    0,  428,    2, 0x08 /* Private */,
      30,    0,  429,    2, 0x08 /* Private */,
      31,    0,  430,    2, 0x08 /* Private */,
      32,    0,  431,    2, 0x08 /* Private */,
      33,    0,  432,    2, 0x08 /* Private */,
      34,    1,  433,    2, 0x08 /* Private */,
      37,    1,  436,    2, 0x08 /* Private */,
      39,    0,  439,    2, 0x08 /* Private */,
      40,    1,  440,    2, 0x08 /* Private */,
      42,    1,  443,    2, 0x08 /* Private */,
      43,    0,  446,    2, 0x08 /* Private */,
      44,    1,  447,    2, 0x08 /* Private */,
      46,    0,  450,    2, 0x08 /* Private */,
      47,    0,  451,    2, 0x08 /* Private */,
      48,    0,  452,    2, 0x08 /* Private */,
      49,    0,  453,    2, 0x08 /* Private */,
      50,    0,  454,    2, 0x08 /* Private */,
      51,    1,  455,    2, 0x08 /* Private */,
      53,    0,  458,    2, 0x08 /* Private */,
      54,    0,  459,    2, 0x08 /* Private */,
      55,    0,  460,    2, 0x08 /* Private */,
      56,    0,  461,    2, 0x08 /* Private */,
      58,    4,  462,    2, 0x08 /* Private */,
      58,    3,  471,    2, 0x28 /* Private | MethodCloned */,
      58,    2,  478,    2, 0x28 /* Private | MethodCloned */,
      65,    0,  483,    2, 0x08 /* Private */,
      66,    1,  484,    2, 0x08 /* Private */,
      68,    0,  487,    2, 0x08 /* Private */,
      69,    0,  488,    2, 0x08 /* Private */,
      71,    0,  489,    2, 0x08 /* Private */,
      72,    1,  490,    2, 0x08 /* Private */,
      74,    0,  493,    2, 0x08 /* Private */,
      75,    0,  494,    2, 0x08 /* Private */,
      76,    1,  495,    2, 0x08 /* Private */,
      79,    0,  498,    2, 0x08 /* Private */,
      80,    1,  499,    2, 0x08 /* Private */,
      82,    2,  502,    2, 0x08 /* Private */,
      85,    0,  507,    2, 0x08 /* Private */,
      86,    0,  508,    2, 0x08 /* Private */,
      87,    1,  509,    2, 0x08 /* Private */,
      88,    0,  512,    2, 0x08 /* Private */,
      89,    0,  513,    2, 0x08 /* Private */,
      90,    0,  514,    2, 0x08 /* Private */,
      91,    1,  515,    2, 0x08 /* Private */,
      93,    0,  518,    2, 0x08 /* Private */,
      94,    0,  519,    2, 0x08 /* Private */,
      96,    1,  520,    2, 0x08 /* Private */,
      98,    0,  523,    2, 0x08 /* Private */,
      99,    1,  524,    2, 0x08 /* Private */,
     101,    1,  527,    2, 0x08 /* Private */,
     102,    1,  530,    2, 0x08 /* Private */,
     104,    0,  533,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    8,
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
    QMetaType::Void, 0x80000000 | 35,   36,
    QMetaType::Void, 0x80000000 | 35,   38,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   41,
    QMetaType::Void, QMetaType::Int,   41,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   45,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 35,   52,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 57,
    0x80000000 | 59, 0x80000000 | 60, 0x80000000 | 60, QMetaType::Double, QMetaType::Double,   61,   62,   63,   64,
    0x80000000 | 59, 0x80000000 | 60, 0x80000000 | 60, QMetaType::Double,   61,   62,   63,
    0x80000000 | 59, 0x80000000 | 60, 0x80000000 | 60,   61,   62,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   67,
    QMetaType::Void,
    0x80000000 | 70,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   73,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 77,   78,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   81,
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   83,   84,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   41,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   92,
    QMetaType::Void,
    0x80000000 | 95,
    QMetaType::Void, 0x80000000 | 95,   97,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  100,
    QMetaType::Void, QMetaType::Bool,   81,
    QMetaType::Void, QMetaType::QString,  103,
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
        case 3: _t->onRobotLoginClicked(); break;
        case 4: _t->onRobotIdChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->onLoginCountdownTick(); break;
        case 6: _t->applyHeightCrop(); break;
        case 7: _t->applyDownsample(); break;
        case 8: _t->computeHull(); break;
        case 9: _t->simplifyPolygon(); break;
        case 10: _t->showPointCloud3D(); break;
        case 11: _t->toggleROISelection(); break;
        case 12: _t->clearROI(); break;
        case 13: _t->toggleObstacleSelection(); break;
        case 14: _t->autoDetectObstacles(); break;
        case 15: _t->deleteSelectedObstacle(); break;
        case 16: _t->clearObstacles(); break;
        case 17: _t->undoSelectionPoint(); break;
        case 18: _t->finishSelection(); break;
        case 19: _t->buildField(); break;
        case 20: _t->generateSwaths(); break;
        case 21: _t->generateRoute(); break;
        case 22: _t->generatePath(); break;
        case 23: _t->clearCoverage(); break;
        case 24: _t->exportPathCSV(); break;
        case 25: _t->publishWaypoints(); break;
        case 26: _t->startNavigation(); break;
        case 27: _t->planHomePath(); break;
        case 28: _t->clearRobotTrail(); break;
        case 29: _t->onPathModeChanged(); break;
        case 30: _t->onROISelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 31: _t->onObstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 32: _t->onSelectionCancelled(); break;
        case 33: _t->onObstacleDeleteRequested((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 34: _t->onObstacleSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 35: _t->onAutoDetectObstaclesFinished(); break;
        case 36: _t->updateDownsampleUI((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 37: _t->tryReconnectROS2(); break;
        case 38: _t->checkZenohBridgeStatus(); break;
        case 39: _t->computeReprojectionError(); break;
        case 40: _t->clearReprojectionError(); break;
        case 41: _t->toggleRectangleMode(); break;
        case 42: _t->onRectangleCompleted((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 43: _t->toggleDarkMode(); break;
        case 44: _t->applyTheme(); break;
        case 45: _t->updateCoverageStats(); break;
        case 46: { CoverageStats _r = _t->computeStats();
            if (_a[0]) *reinterpret_cast< CoverageStats*>(_a[0]) = std::move(_r); }  break;
        case 47: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 48: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 49: { PathStateList _r = _t->computeObstacleAvoidingPath((*reinterpret_cast< const Point2D(*)>(_a[1])),(*reinterpret_cast< const Point2D(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< PathStateList*>(_a[0]) = std::move(_r); }  break;
        case 50: _t->updateWorkflowSteps(); break;
        case 51: _t->onWorkflowStepClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 52: _t->updateLayerVisibility(); break;
        case 53: { QWidget* _r = _t->buildVideoPanelWidget();
            if (_a[0]) *reinterpret_cast< QWidget**>(_a[0]) = std::move(_r); }  break;
        case 54: _t->toggleVideoPanel(); break;
        case 55: _t->onCameraToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 56: _t->playVideoStream(); break;
        case 57: _t->stopVideoStream(); break;
        case 58: _t->onCameraStatusReceived((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 59: _t->openDataTransferDialog(); break;
        case 60: _t->onTransferActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 61: _t->onTransferProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 62: _t->onShowTransferDialogRequested(); break;
        case 63: _t->onCancelTransferRequested(); break;
        case 64: _t->onPresetSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 65: _t->saveCurrentPreset(); break;
        case 66: _t->createNewPreset(); break;
        case 67: _t->openPresetManager(); break;
        case 68: _t->loadPreset((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 69: _t->refreshPresetList(); break;
        case 70: { PlanningPreset _r = _t->gatherCurrentSettings();
            if (_a[0]) *reinterpret_cast< PlanningPreset*>(_a[0]) = std::move(_r); }  break;
        case 71: _t->applyPreset((*reinterpret_cast< const PlanningPreset(*)>(_a[1]))); break;
        case 72: _t->toggleTeleopWidget(); break;
        case 73: _t->onTeleopStatusMessage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 74: _t->onCloudUploadActive((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 75: _t->startScanSession((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 76: _t->endScanSession(); break;
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
        if (_id < 77)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 77;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 77)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 77;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
