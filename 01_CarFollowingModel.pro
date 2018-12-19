SUB_PROJECT_NAME = 01_CarFollowingModel
BASE_DIR = ../../../../../../../..

include (../../../../../../config_type.incl)

SOURCES += \
behavioralModelParticular.cpp \
behavioralModelParticularCreator.cpp \
simVehicleParticular.cpp

HEADERS += \
behavioralModelParticular.h \
behavioralModelParticularCreator.h \
simVehicleParticular.h

DEFINES -= _NANO_AIMSUN
DEFINES += _A2BEHAVIORAL_DEF
LIB_NAME = CarFollowingModel
macx {
	TARGET = $${MACOS_CONTENTS}/Frameworks/$${LIB_NAME}
	QMAKE_LFLAGS_SONAME  = -Wl,-install_name,@rpath/
}else{
	TARGET = $${BIN_PATH}/$${LIB_NAME}
}

CONFIG += warn_on

#--------------------------------------------------------------

TEMPLATE = lib

!defined(AIMSUN_VERSION_NUMBER,var){
	error("AIMSUN_VERSION_NUMBER has not been defined")
}else{
	VERSION = $$AIMSUN_VERSION_NUMBER
}

CONFIG += dll qt thread exceptions rtti

INCLUDEPATH += ../../SDK ../../../../ANG_EXT
DEPENDPATH += $$INCLUDEPATH

win32 {
    LIBS += -L$${BIN_PATH} -la2kernel$${SDK_V_LIB} -lA2BehavioralModelSDK$${SDK_V_LIB}
} macx {
    LIBS += -L$${MACOS_CONTENTS}/Frameworks -lA2BehavioralModelSDK
    LIBS += -L$${MACOS_CONTENTS}/PlugIns -la2kernel
} linux {
    LIBS += -L$${BIN_PATH} -lA2BehavioralModelSDK -la2kernel
}
#--------------------------------------------------------------
