QT -= gui

CONFIG += c++14 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


LIBS += -L/usr/lib/x86_64-linux-gnu -lpcl_io -lpcl_common -lpcl_filters -lpcl_visualization \
        -L/usr/lib/x86_64-linux-gnu -lvtkCommonDataModel-6.3 -lvtkCommonCore-6.3 -lvtkRenderingCore-6.3 -lvtkCommonMath-6.3 \
        -L/usr/lib/x86_64-linux-gnu -lboost_system
INCLUDEPATH += "/usr/include/eigen3/" \
            += "/usr/include/pcl-1.8/" \
            += "/usr/include/vtk-6.3/" \
            += "/usr/include/boost/"
