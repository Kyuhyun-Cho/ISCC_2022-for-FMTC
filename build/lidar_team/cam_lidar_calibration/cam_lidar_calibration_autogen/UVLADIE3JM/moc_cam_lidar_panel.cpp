/****************************************************************************
** Meta object code from reading C++ file 'cam_lidar_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/lidar_team/cam_lidar_calibration/src/cam_lidar_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cam_lidar_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel_t {
    QByteArrayData data[6];
    char stringdata0[88];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel_t qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel = {
    {
QT_MOC_LITERAL(0, 0, 36), // "cam_lidar_calibration::CamLid..."
QT_MOC_LITERAL(1, 37, 13), // "captureSample"
QT_MOC_LITERAL(2, 51, 0), // ""
QT_MOC_LITERAL(3, 52, 13), // "discardSample"
QT_MOC_LITERAL(4, 66, 8), // "optimise"
QT_MOC_LITERAL(5, 75, 12) // "updateResult"

    },
    "cam_lidar_calibration::CamLidarPanel\0"
    "captureSample\0\0discardSample\0optimise\0"
    "updateResult"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cam_lidar_calibration__CamLidarPanel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x09 /* Protected */,
       3,    0,   35,    2, 0x09 /* Protected */,
       4,    0,   36,    2, 0x09 /* Protected */,
       5,    0,   37,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void cam_lidar_calibration::CamLidarPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CamLidarPanel *_t = static_cast<CamLidarPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->captureSample(); break;
        case 1: _t->discardSample(); break;
        case 2: _t->optimise(); break;
        case 3: _t->updateResult(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject cam_lidar_calibration::CamLidarPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel.data,
      qt_meta_data_cam_lidar_calibration__CamLidarPanel,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *cam_lidar_calibration::CamLidarPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cam_lidar_calibration::CamLidarPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cam_lidar_calibration__CamLidarPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz::Panel::qt_metacast(_clname);
}

int cam_lidar_calibration::CamLidarPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
