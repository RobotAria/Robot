TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += C++11

#QMAKE_LFLAGS += /MANIFESTUAC:"level='requireAdministrator'uiAccess='false'"

SOURCES += \
    test.cpp
SOURCES += kinect.cpp
SOURCES += pioneer.cpp
SOURCES += global.cpp

HEADERS += global.h
HEADERS += kinect.h
HEADERS += pioneer.h

LIBS += -pthread

unix:!macx: LIBS += -L$$PWD/../../freenect2/lib/ -lfreenect2

INCLUDEPATH += $$PWD/../../freenect2/lib
DEPENDPATH += $$PWD/../../freenect2/lib

INCLUDEPATH += $$PWD/../../freenect2/include/libfreenect2
INCLUDEPATH += $$PWD/../../freenect2/include

unix:!macx: LIBS += -L/usr/local/Aria/lib/ -lAria

INCLUDEPATH += /usr/local/Aria/include
DEPENDPATH += /usr/local/Aria/include

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gpu

unix:!macx: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system

unix:!macx: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_filesystem

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu


INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3

unix:!macx: LIBS += -L/usr/lib/ -lpcl_io -lpcl_common -lpcl_visualization

INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

INCLUDEPATH += /usr/include/vtk-6.2
DEPENDPATH += /usr/include/vtk-6.2
