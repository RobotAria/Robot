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

INCLUDEPATH += $$PWD/../freenect2/lib
DEPENDPATH += $$PWD/../freenect2/lib

INCLUDEPATH += $$PWD/../../freenect2/include/libfreenect2
INCLUDEPATH += $$PWD/../../freenect2/include

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/Aria/lib/ -lAria

INCLUDEPATH += $$PWD/../../../../usr/local/Aria/include
DEPENDPATH += $$PWD/../../../../usr/local/Aria/include

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gpu

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lboost_system

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lboost_filesystem

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

#unix:!macx: LIBS += -L$$PWD/../../../../usr/local/cuda-7.5/lib64/ -lcudart

#INCLUDEPATH += $$PWD/../../../../usr/local/cuda-7.5/lib64
#DEPENDPATH += $$PWD/../../../../usr/local/cuda-7.5/lib64

INCLUDEPATH += $$PWD/../../../../usr/include/eigen3
DEPENDPATH += $$PWD/../../../../usr/include/eigen3

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_io -lpcl_common -lpcl_visualization

INCLUDEPATH += $$PWD/../../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../../usr/include/pcl-1.7

INCLUDEPATH += $$PWD/../../../../usr/include/vtk-6.2
DEPENDPATH += $$PWD/../../../../usr/include/vtk-6.2
