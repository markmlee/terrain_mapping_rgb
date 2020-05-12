/****************************************************************************
** Meta object code from reading C++ file 'ROSThread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/ALPrograms/PODOLAN/ROSThread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ROSThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ROSWorker_t {
    QByteArrayData data[18];
    char stringdata0[203];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSWorker_t qt_meta_stringdata_ROSWorker = {
    {
QT_MOC_LITERAL(0, 0, 9), // "ROSWorker"
QT_MOC_LITERAL(1, 10, 13), // "ROS_Connected"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 16), // "ROS_Disconnected"
QT_MOC_LITERAL(4, 42, 13), // "RST_Connected"
QT_MOC_LITERAL(5, 56, 16), // "RST_Disconnected"
QT_MOC_LITERAL(6, 73, 12), // "ROS_CMD_READ"
QT_MOC_LITERAL(7, 86, 11), // "R2P_command"
QT_MOC_LITERAL(8, 98, 4), // "_cmd"
QT_MOC_LITERAL(9, 103, 12), // "ROSConnected"
QT_MOC_LITERAL(10, 116, 15), // "ROSDisconnected"
QT_MOC_LITERAL(11, 132, 12), // "RSTConnected"
QT_MOC_LITERAL(12, 145, 15), // "RSTDisconnected"
QT_MOC_LITERAL(13, 161, 7), // "readCMD"
QT_MOC_LITERAL(14, 169, 5), // "char*"
QT_MOC_LITERAL(15, 175, 5), // "_data"
QT_MOC_LITERAL(16, 181, 10), // "sendSTATUS"
QT_MOC_LITERAL(17, 192, 10) // "sendRESULT"

    },
    "ROSWorker\0ROS_Connected\0\0ROS_Disconnected\0"
    "RST_Connected\0RST_Disconnected\0"
    "ROS_CMD_READ\0R2P_command\0_cmd\0"
    "ROSConnected\0ROSDisconnected\0RSTConnected\0"
    "RSTDisconnected\0readCMD\0char*\0_data\0"
    "sendSTATUS\0sendRESULT"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x06 /* Public */,
       3,    0,   75,    2, 0x06 /* Public */,
       4,    0,   76,    2, 0x06 /* Public */,
       5,    0,   77,    2, 0x06 /* Public */,
       6,    1,   78,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    1,   85,    2, 0x08 /* Private */,
      16,    0,   88,    2, 0x08 /* Private */,
      17,    0,   89,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ROSWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ROSWorker *_t = static_cast<ROSWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ROS_Connected(); break;
        case 1: _t->ROS_Disconnected(); break;
        case 2: _t->RST_Connected(); break;
        case 3: _t->RST_Disconnected(); break;
        case 4: _t->ROS_CMD_READ((*reinterpret_cast< R2P_command(*)>(_a[1]))); break;
        case 5: _t->ROSConnected(); break;
        case 6: _t->ROSDisconnected(); break;
        case 7: _t->RSTConnected(); break;
        case 8: _t->RSTDisconnected(); break;
        case 9: _t->readCMD((*reinterpret_cast< char*(*)>(_a[1]))); break;
        case 10: _t->sendSTATUS(); break;
        case 11: _t->sendRESULT(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ROSWorker::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSWorker::ROS_Connected)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (ROSWorker::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSWorker::ROS_Disconnected)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (ROSWorker::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSWorker::RST_Connected)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (ROSWorker::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSWorker::RST_Disconnected)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (ROSWorker::*_t)(R2P_command );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSWorker::ROS_CMD_READ)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject ROSWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ROSWorker.data,
      qt_meta_data_ROSWorker,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSWorker.stringdata0))
        return static_cast<void*>(const_cast< ROSWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int ROSWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void ROSWorker::ROS_Connected()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void ROSWorker::ROS_Disconnected()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void ROSWorker::RST_Connected()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void ROSWorker::RST_Disconnected()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void ROSWorker::ROS_CMD_READ(R2P_command _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
struct qt_meta_stringdata_ROSServer_t {
    QByteArrayData data[6];
    char stringdata0[43];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSServer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSServer_t qt_meta_stringdata_ROSServer = {
    {
QT_MOC_LITERAL(0, 0, 9), // "ROSServer"
QT_MOC_LITERAL(1, 10, 10), // "ROS_UPDATE"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 5), // "char*"
QT_MOC_LITERAL(4, 28, 5), // "_data"
QT_MOC_LITERAL(5, 34, 8) // "ReadData"

    },
    "ROSServer\0ROS_UPDATE\0\0char*\0_data\0"
    "ReadData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSServer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   27,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void ROSServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ROSServer *_t = static_cast<ROSServer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ROS_UPDATE((*reinterpret_cast< char*(*)>(_a[1]))); break;
        case 1: _t->ReadData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ROSServer::*_t)(char * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ROSServer::ROS_UPDATE)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject ROSServer::staticMetaObject = {
    { &RBTCPServer::staticMetaObject, qt_meta_stringdata_ROSServer.data,
      qt_meta_data_ROSServer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ROSServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSServer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ROSServer.stringdata0))
        return static_cast<void*>(const_cast< ROSServer*>(this));
    return RBTCPServer::qt_metacast(_clname);
}

int ROSServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RBTCPServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void ROSServer::ROS_UPDATE(char * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_RSTServer_t {
    QByteArrayData data[6];
    char stringdata0[43];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RSTServer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RSTServer_t qt_meta_stringdata_RSTServer = {
    {
QT_MOC_LITERAL(0, 0, 9), // "RSTServer"
QT_MOC_LITERAL(1, 10, 10), // "ROS_UPDATE"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 5), // "char*"
QT_MOC_LITERAL(4, 28, 5), // "_data"
QT_MOC_LITERAL(5, 34, 8) // "ReadData"

    },
    "RSTServer\0ROS_UPDATE\0\0char*\0_data\0"
    "ReadData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RSTServer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   27,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void RSTServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RSTServer *_t = static_cast<RSTServer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ROS_UPDATE((*reinterpret_cast< char*(*)>(_a[1]))); break;
        case 1: _t->ReadData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RSTServer::*_t)(char * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RSTServer::ROS_UPDATE)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject RSTServer::staticMetaObject = {
    { &RBTCPServer::staticMetaObject, qt_meta_stringdata_RSTServer.data,
      qt_meta_data_RSTServer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RSTServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RSTServer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RSTServer.stringdata0))
        return static_cast<void*>(const_cast< RSTServer*>(this));
    return RBTCPServer::qt_metacast(_clname);
}

int RSTServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RBTCPServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void RSTServer::ROS_UPDATE(char * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
