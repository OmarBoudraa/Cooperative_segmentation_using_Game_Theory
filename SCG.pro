#-------------------------------------------------
#
# Project created by QtCreator 2014-03-17T20:26:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
qtHaveModule(printsupport): QT += printsupport
CONFIG +=debug
QMAKE_LFLAGS_WINDOWS += -Wl,--stack,32000000
TARGET = SCG
TEMPLATE = app
#opencv include files
INCLUDEPATH += "C:\openc-mingw\install\include"

#INCLUDEPATH += "C:\QT-project\install\include\opencv2"
#INCLUDEPATH += "C:\openc-mingw\install\include\opencv2"

#opencv libs
LIBS += -L"C:\openc-mingw\install\x64\mingw\bin"
LIBS += -lopencv_calib3d248 -lopencv_contrib248 -lopencv_features2d248 -lopencv_core248  -lopencv_features2d248 -lopencv_flann248 -lopencv_gpu248 -lopencv_highgui248 -lopencv_imgproc248 -lopencv_legacy248 -lopencv_ml248 -lopencv_nonfree248 -lopencv_objdetect248 -lopencv_ocl248 -lopencv_photo248 -lopencv_stitching248 -lopencv_superres248 -lopencv_video248 -lopencv_videostab248  # here just use dll file names without "lib" just add "-l" in front


SOURCES += main.cpp\
        principale.cpp \
    statistique.cpp \
    segmentation.cpp \
    croissanceregion.cpp \
    snakes.cpp \
    guiscg.cpp \
    evaluationseg.cpp \
    guisegeval.cpp

HEADERS  += principale.h \
    statistique.h \
    segmentation.h \
    croissanceregion.h \
    snakes.h \
    guiscg.h \
    evaluationseg.h \
    guisegeval.h

FORMS    += principale.ui \
    croissanceregion.ui \
    snakes.ui \
    guiscg.ui \
    guisegeval.ui
