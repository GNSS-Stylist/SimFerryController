QT       += core gui
QT += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Following two defines remove Eigen's vectorization that seems to cause
# runtime assertion failure (not always, but sometimes, how?!?)
# By default it tries to use fast (SIMD) instructions for calculation, see:
# http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
# Here lack of vectorization may not be a big issue.
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    MiniPID/MiniPID.cpp \
    autopilot.cpp \
    losolver.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    MiniPID/MiniPID.h \
    autopilot.h \
    losolver.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
