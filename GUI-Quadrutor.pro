#-------------------------------------------------
#
# Project created by QtCreator 2018-01-24T13:52:25
#
#-------------------------------------------------

QT       += core gui printsupport
QT +=3dcore 3drender 3dinput 3dextras
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GUI-Quadrutor
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    utils.cpp \
    quad.cpp \
    controller.cpp \
    scenemodifier.cpp \
    pso.cpp \
    qcustomplot.cpp \
    plot.cpp

HEADERS  += mainwindow.h \
    utils.h \
    quad.h \
    controller.h \
    scenemodifier.h \
    pso.h \
    qcustomplot.h \
    plot.h

FORMS    += mainwindow.ui

DISTFILES +=
QMAKE_CXXFLAGS += -std=gnu++14
