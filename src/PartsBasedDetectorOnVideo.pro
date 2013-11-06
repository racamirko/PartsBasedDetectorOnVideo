#-------------------------------------------------
#
# Project created by QtCreator 2013-11-05T16:13:43
#
#-------------------------------------------------

QT       -= gui

TARGET = PartsBasedDetectorOnVideo
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

QMAKE_CXXFLAGS_DEBUG += -g
QMAKE_CXXFLAGS_RELEASE += -O3 -march=native -DNDEBUG
QMAKE_CXXFLAGS += -std=c++0x

INCLUDEPATH += /usr/local/include /home/raca/repo/outside_projects/PartsBasedDetector/include

LIBS += -L/usr/lib -L/usr/local/lib -lPartsBasedDetector -lopencv_core -lopencv_highgui -lglog -lboost_filesystem -lboost_system

SOURCES += main.cpp \
    outputFormat.cpp \
    mirrorUtils.cpp

HEADERS += \
    outputFormat.h \
    mirrorUtils.h
