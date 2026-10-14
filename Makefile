CXX         = g++
CXXFLAGS    = -std=c++17 -Wall -Wextra -O2
SRC         = src/main.cpp src/lander.cpp src/logger.cpp
HDR         = src/params.hpp src/lander.hpp src/logger.hpp src/guidance.hpp src/pid.hpp src/controller.hpp
TARGET      = build/lander.exe
PHYS_TEST   = build/test_physics.exe
GUIDE_TEST  = build/test_guidance.exe
PID_TEST    = build/test_pid.exe
CTRL_TEST   = build/test_controller.exe

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test: $(PHYS_TEST) $(GUIDE_TEST) $(PID_TEST) $(CTRL_TEST)
	./$(PHYS_TEST)
	./$(GUIDE_TEST)
	./$(PID_TEST)
	./$(CTRL_TEST)

$(PHYS_TEST): tests/test_physics.cpp src/lander.cpp $(HDR)
	$(CXX) $(CXXFLAGS) -Isrc tests/test_physics.cpp src/lander.cpp -o $(PHYS_TEST)

$(GUIDE_TEST): tests/test_guidance.cpp src/guidance.cpp $(HDR)
	$(CXX) $(CXXFLAGS) -Isrc tests/test_guidance.cpp src/guidance.cpp -o $(GUIDE_TEST)

$(PID_TEST): tests/test_pid.cpp src/pid.cpp $(HDR)
	$(CXX) $(CXXFLAGS) -Isrc tests/test_pid.cpp src/pid.cpp -o $(PID_TEST)

$(CTRL_TEST): tests/test_controller.cpp src/controller.cpp src/pid.cpp $(HDR)
	$(CXX) $(CXXFLAGS) -Isrc tests/test_controller.cpp src/controller.cpp src/pid.cpp -o $(CTRL_TEST)