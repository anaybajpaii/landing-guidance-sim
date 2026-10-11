CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
SRC      = src/main.cpp src/lander.cpp src/logger.cpp
HDR      = src/params.hpp src/lander.hpp src/logger.hpp
TARGET   = build/lander.exe

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)