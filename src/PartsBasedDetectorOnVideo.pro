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

LIBS += -L/usr/local/lib -L/usr/lib -lPartsBasedDetector -lopencv_core -lopencv_highgui -lglog -lboost_filesystem -lboost_system -lncurses -lboost_program_options -lopencv_imgproc

SOURCES += main.cpp \
    outputFormat.cpp \
    mirrorUtils.cpp \
    filters/FilterSize.cpp \
    filters/FilterNMS.cpp \
    filters/PreFilterBackgroundMask.cpp \
    dataprovider/CVideoFrameProvider.cpp \
    dataprovider/CProviderFactory.cpp

HEADERS += \
    outputFormat.h \
    mirrorUtils.h \
    filters/FilterSize.h \
    globalIncludes.h \
    filters/FilterNMS.h \
    filters/GenericPostFilter.h \
    filters/GenericPreFilter.h \
    filters/PreFilterBackgroundSub.h \
    filters/PreFilterBackgroundMask.h \
    dataprovider/CGenericFrameProvider.h \
    dataprovider/CVideoFrameProvider.h \
    dataprovider/CProviderFactory.h
