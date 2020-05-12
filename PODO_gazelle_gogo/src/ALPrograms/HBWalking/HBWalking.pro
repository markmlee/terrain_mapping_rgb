QT       += core

QT       -= gui

TARGET = HBWalking

TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle


CONFIG(debug, debug|release) {
    DESTDIR = ../PODO_PROC_Build
} else {
    DESTDIR = ../PODO_PROC_Build
}

QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    ../../../share/Headers\
    /usr/include/eigen3\
    /usr/local/lib

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L/usr/local/lib/ -lrbdl


SOURCES += main.cpp \
    taskmotion.cpp \
    BasicTrajectory.cpp \
    HB_functions.cpp \
    ManualCAN.cpp \
    Array.cc \
    QuadProg++.cc \
    HB_PreviewWalk.cpp \
    HB_Controller.cpp \
    robotstateestimator.cpp \
    basicsetting.cpp \
    GG_SingleLogWalk.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    BasicMath.h \
    BasicMatrix.h \
    HB_functions.h \
    HB_walking.h \
    Oinverse.h \
    taskmotion.h \
    BasicTrajectory.h \
    taskGeneral.h \
    ManualCAN.h \
    HB_inverse.h \
    BP_RBDL.h \
    HB_dynamics_walk.h \
    HB_types.h \
    HB_State_est.h \
    ow_cplex.h \
    Array.hh \
    QuadProg++.hh \
    HB_dynamics_walk2.h \
    HB_Jumping.h \
    HB_PreviewWalk.h \
    HB_StepAdjustor.h \
    robotstateestimator_parameter_Gazelle.h \
    robotstateestimator.h \
    GG_SingleLogWalk.h


