CXX         = g++
CXXFLAGS    = -std=c++17 -Wall -Wextra -O2
SRC         = src/main.cpp src/lander.cpp src/logger.cpp
HDR         = src/params.hpp src/lander.hpp src/logger.hpp
TARGET      = build/lander.exe
TEST_TARGET = build/test_physics.exe

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): tests/test_physics.cpp src/lander.cpp $(HDR)
	$(CXX) $(CXXFLAGS) -Isrc tests/test_physics.cpp src/lander.cpp -o $(TEST_TARGET)