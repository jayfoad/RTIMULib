CXX = arm-linux-gnueabihf-g++
CXXFLAGS = -g -O2

SRCS = \
	CWrapper.cpp \
	RTFusion.cpp \
	RTFusionKalman4.cpp \
	RTFusionRTQF.cpp \
	RTIMU.cpp \
	RTIMUGD20HM303D.cpp \
	RTIMUGD20M303DLHC.cpp \
	RTIMUHal.cpp \
	RTIMULSM9DS0.cpp \
	RTIMUMPU9150.cpp \
	RTIMUNull.cpp \
	RTIMUSettings.cpp \
	RTMath.cpp

libRTIMULib.so : $(SRCS)
	$(CXX) $(CXXFLAGS) -fvisibility=hidden -shared -fPIC -lpthread -o $@ $^
