#-------------------------------------------------
#
# Project created by QtCreator 2017-06-13T17:07:30
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = ExtendedKF
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

QMAKE_CXX=g++-5
QMAKE_CXXFLAGS += -std=c++11 -Wall

DESTDIR = ../

INCLUDEPATH += ../../src

SOURCES += \
    ../../src/FusionEKF.cpp \
    ../../src/kalman_filter.cpp \
    ../../src/main.cpp \
    ../../src/tools.cpp

HEADERS += \
    ../../src/FusionEKF.h \
    ../../src/json.hpp \
    ../../src/kalman_filter.h \
    ../../src/measurement_package.h \
    ../../src/tools.h

LIBS += -lz -lssl -luv -luWS
