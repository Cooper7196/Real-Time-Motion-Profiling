# Compiler and flags
CXX := g++
CXXFLAGS := -Wall -Wextra -Iinclude -std=c++17

# Directories
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build

# Files
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRCS))
TARGET := main

# Default target
all: $(TARGET)

# Link objects into the final binary
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@

# Compile each .cpp into .o inside build/
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Make sure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean up
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Rebuild everything
rebuild: clean all

.PHONY: all clean rebuild
