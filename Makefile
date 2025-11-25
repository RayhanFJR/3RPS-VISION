# Makefile for 3RPS Modular System
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
INCLUDES = -Iinclude
LIBS = -lmodbus -lboost_system -pthread

# Directories
SRC_DIR = src
OBJ_DIR = obj
INCLUDE_DIR = include

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Target executable
TARGET = mainhmi

# Default target
all: $(TARGET)

# Create object directory
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Build object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Link executable
$(TARGET): $(OBJECTS) main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) main.cpp $(OBJECTS) -o $(TARGET) $(LIBS)

# Clean build files
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Install (optional)
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/

.PHONY: all clean install

