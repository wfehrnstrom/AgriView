#-------------------------------------------------
#
# Project created by QtCreator 2016-09-14T20:00:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AgriGate
TEMPLATE = app


INCLUDEPATH += /Users/wfehrnstrom/vtk-install/include/vtk-7.0 \
               /usr/local/include/pdal \
               /usr/local/include/pcl-1.8 \
               /usr/local/Cellar/boost/1.61.0_1/include \
               /usr/local/Cellar/eigen/3.2.9/include/eigen3 \
               /usr/local/Cellar/flann/1.8.4_1/include


SOURCES += main.cpp\
        agrigatemainwindow.cpp \
    map.cpp \
    lidarfile.cpp

HEADERS  += agrigatemainwindow.h \
    map.h \
    lidarfile.h \
    pointxyzrgbai.h

LIBS += -L/Users/wfehrnstrom/vtk-install/lib
LIBS += -L/usr/local/Cellar/boost/1.61.0_1/lib
LIBS += -lQVTKWidgetPlugin \
     -lvtkGUISupportQt-7.0.1 \
     -lvtkInteractionStyle-7.0.1 \
     -lvtkInteractionWidgets-7.0.1 \
     -lvtkRenderingOpenGL2-7.0.1 \
     -lvtkViewsQt-7.0.1 \
     -lvtkViewsCore-7.0.1 \
     -lvtkRenderingCore-7.0.1 \
     -lvtkCommonCore-7.0.1 \
     -lvtksys-7.0.1 \
     -lvtkImagingCore-7.0.1 \
     -lvtkFiltersCore-7.0.1 \
     -lvtkIOCore-7.0.1 \
     -lvtkChartsCore-7.0.1 \
     -lvtkInfovisCore-7.0.1 \
     -lvtkInfovisLayout-7.0.1 \
     -lvtkglew-7.0.1 \
     -lvtkalglib-7.0.1 \
     -lvtkzlib-7.0.1 \
     -lvtkmetaio-7.0.1 \
     -lvtkfreetype-7.0.1 \
     -lvtkIOPLY-7.0.1



LIBS += -L/usr/local/lib \
        -lpcl_common \
        -lpcl_io \
        -lpcl_features \
        -lpcl_visualization


FORMS    += agrigatemainwindow.ui
