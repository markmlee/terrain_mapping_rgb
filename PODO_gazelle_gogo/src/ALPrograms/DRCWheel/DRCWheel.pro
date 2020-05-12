TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle


QMAKE_CXXFLAGS += -I/usr/xenomai/include/cobalt
QMAKE_CXXFLAGS += -I/usr/xenomai/include
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/xenomai/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/xenomai/include/cobalt \
    /usr/xenomai/include \
    /usr/xenomai/include/alchemy \
    ../../SHARE/Headers

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -L/usr/xenomai/lib \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L../../SHARE/Libs      -lKINE_DRC_HUBO2 \
    -L../../SHARE/Libs      -lik_math2 \


SOURCES += main.cpp \
    BasicFiles/BasicTrajectory.cpp \
    BasicFiles/taskmotion.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    BasicFiles/BasicTrajectory.h \
    BasicFiles/taskmotion.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \
    BasicFiles/taskGeneral.h
