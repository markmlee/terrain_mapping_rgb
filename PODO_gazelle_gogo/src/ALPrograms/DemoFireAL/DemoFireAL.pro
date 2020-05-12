
QT       += core
QT       -= gui

TARGET = DemoFireAL
CONFIG   += console
CONFIG   -= app_bundle

CONFIG(debug, debug|release) {
	DESTDIR = ../PODO_PROC_Build
} else {
	DESTDIR = ../PODO_PROC_Build
}

TEMPLATE = app


QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt

INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy

LIBS        += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
   -L../../../share/Libs       -lKINE_DRC_HUBO3 \
   -L../../../share/Libs	-lik_math3

SOURCES += main.cpp \
	joint.cpp \
	taskmotion.cpp \
	BasicTrajectory.cpp \
    DebrisMotion.cpp \
    DebrisMotionScript.cpp



HEADERS += \
	joint.h \
	taskmotion.h \
	taskGeneral.h \
	BasicTrajectory.h \
	BasicMatrix.h \
	BasicMath.h \
	RBLog.h \
    DebrisMotion.h
