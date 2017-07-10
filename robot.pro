TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp
#LIBS += -fpermissive
LIBS += -ldl
LIBS += -lpthread
unix:!macx: LIBS += -L /usr/local/Aria/lib/ -lAria
INCLUDEPATH += /usr/local/Aria/include
DEPENDPATH += /usr/local/Aria/include
