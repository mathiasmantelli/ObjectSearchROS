TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++0x

SOURCES += \
    thinning.cpp \
    Kernel.cpp \
    SomeKernels.cpp \
    Utils.cpp \
    Grid.cpp \
    PioneerBase.cpp \
    PioneerBase_ARIA.cpp \
    GlutClass.cpp \
    BVP.cpp \
    Planning.cpp \
    Robot.cpp \
    GeoAssistant.cpp \
    main.cpp \
    Doors.cpp \
    SemanticBVP.cpp

OTHER_FILES += \
    CONTROLE.txt \
    inf73supdoors.txt

HEADERS += \
    thinning.h \
    Kernel.h \
    SomeKernels.h \
    Utils.h \
    Grid.h \
    PioneerBase.h \
    PioneerBase_ARIA.h \
    GlutClass.h \
    BVP.h \
    Planning.h \
    Robot.h \
    GeoAssistant.h \
    Doors.h \
    SemanticBVP.h

INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/Aria/lib -lAria
#INCLUDEPATH+=../ARIA/Aria-2.7.2/include
#LIBS+=-L../ARIA/Aria-2.7.2/lib -lAria

LIBS+=-lpthread -lglut -ldl -lrt -lGL -lfreeimage
