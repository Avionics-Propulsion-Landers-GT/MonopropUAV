CXX = g++
CXXFLAGS = -std=c++17 -g -O0 -fsanitize=undefined -fno-omit-frame-pointer
LDFLAGS = -lgfortran
TARGET = data_mocker

SOURCES = \
    data_mocker.cpp \
    Loop/loop.cpp \
    Loop/init.cpp \
    Filters/Madgwick/updatedMadgwick.cpp \
    Filters/EKFs/ExtendedKalmanFilterGeneral.cpp \
    Filters/EKFs/EKF_xy.cpp \
    Filters/EKFs/EKF_z.cpp \
    LQR/lqr.cpp \
    CustomLinear/Vector.cpp \
    CustomLinear/Matrix.cpp \
    LQR/calculateA.cpp \
    LQR/calculateB.cpp \
    LQR/solveCARE.cpp

STATIC_LIBS = lapack/lapack_routine/libmylapack.a

OBJECTS = $(SOURCES:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(STATIC_LIBS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJECTS)
