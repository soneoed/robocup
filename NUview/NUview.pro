QT += network \
    opengl
macx { 
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework \
        CoreFoundation \
        -L \
        /usr/lib/libz.dylib
}
INCLUDEPATH += ../
HEADERS += ui_mainwindow.h \
    mainwindow.h \
    connectionwidget.h \
    ColorModelConversions.h \
    classificationwidget.h \
    ClassificationColours.h \
    ../Tools/FileFormats/NUbotImage.h \
    ../Vision/Vision.h \
    ../Tools/FileFormats/LUTTools.h \
    virtualnubot.h \
    ../Tools/Image/BresenhamLine.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Line.h \
    ../Kinematics/Horizon.h \
    openglmanager.h \
    GLDisplay.h \
    ../Tools/Image/NUimage.h \
    ../Tools/Image/ClassifiedImage.h
SOURCES += mainwindow.cpp \
    main.cpp \
    connectionwidget.cpp \
    classificationwidget.cpp \
    ../Tools/FileFormats/NUbotImage.cpp \
    ../Vision/Vision.cpp \
    ../Tools/FileFormats/LUTTools.cpp \
    virtualnubot.cpp \
    ../Tools/Image/BresenhamLine.cpp \
    ../Tools/Math/Line.cpp \
    ../Kinematics/Horizon.cpp \
    openglmanager.cpp \
    GLDisplay.cpp \
    gl/GLee.c \
    ../Tools/Image/NUimage.cpp \
    ../Tools/Image/ClassifiedImage.cpp