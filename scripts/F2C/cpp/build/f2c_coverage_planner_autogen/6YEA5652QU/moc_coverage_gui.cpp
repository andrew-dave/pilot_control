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
struct qt_meta_stringdata_f2c_cpp__PlotWidget_t {
    QByteArrayData data[11];
    char stringdata0[130];
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
QT_MOC_LITERAL(8, 92, 23), // "customWaypointRequested"
QT_MOC_LITERAL(9, 116, 7), // "Point2D"
QT_MOC_LITERAL(10, 124, 5) // "point"

    },
    "f2c_cpp::PlotWidget\0roiSelected\0\0"
    "Polygon2D\0roi\0obstacleSelected\0obstacle\0"
    "selectionCancelled\0customWaypointRequested\0"
    "Point2D\0point"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__PlotWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,
       7,    0,   40,    2, 0x06 /* Public */,
       8,    1,   41,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    6,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,

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
        case 3: _t->customWaypointRequested((*reinterpret_cast< const Point2D(*)>(_a[1]))); break;
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
            using _t = void (PlotWidget::*)(const Point2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotWidget::customWaypointRequested)) {
                *result = 3;
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
void f2c_cpp::PlotWidget::customWaypointRequested(const Point2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
struct qt_meta_stringdata_f2c_cpp__CoverageGUI_t {
    QByteArrayData data[39];
    char stringdata0[604];
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
QT_MOC_LITERAL(6, 89, 15), // "applyHeightCrop"
QT_MOC_LITERAL(7, 105, 15), // "applyDownsample"
QT_MOC_LITERAL(8, 121, 11), // "computeHull"
QT_MOC_LITERAL(9, 133, 15), // "simplifyPolygon"
QT_MOC_LITERAL(10, 149, 16), // "showPointCloud3D"
QT_MOC_LITERAL(11, 166, 18), // "toggleROISelection"
QT_MOC_LITERAL(12, 185, 8), // "clearROI"
QT_MOC_LITERAL(13, 194, 23), // "toggleObstacleSelection"
QT_MOC_LITERAL(14, 218, 14), // "clearObstacles"
QT_MOC_LITERAL(15, 233, 18), // "undoSelectionPoint"
QT_MOC_LITERAL(16, 252, 15), // "finishSelection"
QT_MOC_LITERAL(17, 268, 10), // "buildField"
QT_MOC_LITERAL(18, 279, 14), // "generateSwaths"
QT_MOC_LITERAL(19, 294, 13), // "generateRoute"
QT_MOC_LITERAL(20, 308, 12), // "generatePath"
QT_MOC_LITERAL(21, 321, 13), // "clearCoverage"
QT_MOC_LITERAL(22, 335, 13), // "exportPathCSV"
QT_MOC_LITERAL(23, 349, 16), // "publishWaypoints"
QT_MOC_LITERAL(24, 366, 15), // "startNavigation"
QT_MOC_LITERAL(25, 382, 15), // "clearRobotTrail"
QT_MOC_LITERAL(26, 398, 17), // "onPathModeChanged"
QT_MOC_LITERAL(27, 416, 13), // "onROISelected"
QT_MOC_LITERAL(28, 430, 9), // "Polygon2D"
QT_MOC_LITERAL(29, 440, 3), // "roi"
QT_MOC_LITERAL(30, 444, 18), // "onObstacleSelected"
QT_MOC_LITERAL(31, 463, 8), // "obstacle"
QT_MOC_LITERAL(32, 472, 20), // "onSelectionCancelled"
QT_MOC_LITERAL(33, 493, 18), // "updateDownsampleUI"
QT_MOC_LITERAL(34, 512, 6), // "method"
QT_MOC_LITERAL(35, 519, 16), // "tryReconnectROS2"
QT_MOC_LITERAL(36, 536, 19), // "onDdsProfileChanged"
QT_MOC_LITERAL(37, 556, 24), // "computeReprojectionError"
QT_MOC_LITERAL(38, 581, 22) // "clearReprojectionError"

    },
    "f2c_cpp::CoverageGUI\0loadPointCloud\0"
    "\0fetchLatestMapFromRobot\0"
    "loadPointCloudFromPath\0path\0applyHeightCrop\0"
    "applyDownsample\0computeHull\0simplifyPolygon\0"
    "showPointCloud3D\0toggleROISelection\0"
    "clearROI\0toggleObstacleSelection\0"
    "clearObstacles\0undoSelectionPoint\0"
    "finishSelection\0buildField\0generateSwaths\0"
    "generateRoute\0generatePath\0clearCoverage\0"
    "exportPathCSV\0publishWaypoints\0"
    "startNavigation\0clearRobotTrail\0"
    "onPathModeChanged\0onROISelected\0"
    "Polygon2D\0roi\0onObstacleSelected\0"
    "obstacle\0onSelectionCancelled\0"
    "updateDownsampleUI\0method\0tryReconnectROS2\0"
    "onDdsProfileChanged\0computeReprojectionError\0"
    "clearReprojectionError"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_f2c_cpp__CoverageGUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      32,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  174,    2, 0x08 /* Private */,
       3,    0,  175,    2, 0x08 /* Private */,
       4,    1,  176,    2, 0x08 /* Private */,
       6,    0,  179,    2, 0x08 /* Private */,
       7,    0,  180,    2, 0x08 /* Private */,
       8,    0,  181,    2, 0x08 /* Private */,
       9,    0,  182,    2, 0x08 /* Private */,
      10,    0,  183,    2, 0x08 /* Private */,
      11,    0,  184,    2, 0x08 /* Private */,
      12,    0,  185,    2, 0x08 /* Private */,
      13,    0,  186,    2, 0x08 /* Private */,
      14,    0,  187,    2, 0x08 /* Private */,
      15,    0,  188,    2, 0x08 /* Private */,
      16,    0,  189,    2, 0x08 /* Private */,
      17,    0,  190,    2, 0x08 /* Private */,
      18,    0,  191,    2, 0x08 /* Private */,
      19,    0,  192,    2, 0x08 /* Private */,
      20,    0,  193,    2, 0x08 /* Private */,
      21,    0,  194,    2, 0x08 /* Private */,
      22,    0,  195,    2, 0x08 /* Private */,
      23,    0,  196,    2, 0x08 /* Private */,
      24,    0,  197,    2, 0x08 /* Private */,
      25,    0,  198,    2, 0x08 /* Private */,
      26,    0,  199,    2, 0x08 /* Private */,
      27,    1,  200,    2, 0x08 /* Private */,
      30,    1,  203,    2, 0x08 /* Private */,
      32,    0,  206,    2, 0x08 /* Private */,
      33,    1,  207,    2, 0x08 /* Private */,
      35,    0,  210,    2, 0x08 /* Private */,
      36,    0,  211,    2, 0x08 /* Private */,
      37,    0,  212,    2, 0x08 /* Private */,
      38,    0,  213,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
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
    QMetaType::Void, 0x80000000 | 28,   29,
    QMetaType::Void, 0x80000000 | 28,   31,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   34,
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
        case 3: _t->applyHeightCrop(); break;
        case 4: _t->applyDownsample(); break;
        case 5: _t->computeHull(); break;
        case 6: _t->simplifyPolygon(); break;
        case 7: _t->showPointCloud3D(); break;
        case 8: _t->toggleROISelection(); break;
        case 9: _t->clearROI(); break;
        case 10: _t->toggleObstacleSelection(); break;
        case 11: _t->clearObstacles(); break;
        case 12: _t->undoSelectionPoint(); break;
        case 13: _t->finishSelection(); break;
        case 14: _t->buildField(); break;
        case 15: _t->generateSwaths(); break;
        case 16: _t->generateRoute(); break;
        case 17: _t->generatePath(); break;
        case 18: _t->clearCoverage(); break;
        case 19: _t->exportPathCSV(); break;
        case 20: _t->publishWaypoints(); break;
        case 21: _t->startNavigation(); break;
        case 22: _t->clearRobotTrail(); break;
        case 23: _t->onPathModeChanged(); break;
        case 24: _t->onROISelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 25: _t->onObstacleSelected((*reinterpret_cast< const Polygon2D(*)>(_a[1]))); break;
        case 26: _t->onSelectionCancelled(); break;
        case 27: _t->updateDownsampleUI((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 28: _t->tryReconnectROS2(); break;
        case 29: _t->onDdsProfileChanged(); break;
        case 30: _t->computeReprojectionError(); break;
        case 31: _t->clearReprojectionError(); break;
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
        if (_id < 32)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 32;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 32)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 32;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
