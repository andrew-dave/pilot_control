# Makefile for Seek Thermal Camera Viewer

CXX = g++
CXXFLAGS = -std=c++17 -O3 -march=native -Wall -Wextra
INCLUDES = -I/usr/include/seekcamera -I/usr/include/seekframe $(shell pkg-config --cflags opencv4)
LIBS = -lseekcamera -lseekware -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lpthread

TARGET = seek_thermal_viewer
TARGET_ENHANCED = seek_thermal_viewer_enhanced
TARGET_SIMPLE = seek_thermal_simple
TARGET_CHECK = check_camera_settings
SOURCE = seek_thermal_viewer.cpp
SOURCE_ENHANCED = seek_thermal_viewer_enhanced.cpp
SOURCE_SIMPLE = seek_thermal_simple.cpp
SOURCE_CHECK = check_camera_settings.cpp

.PHONY: all clean install enhanced simple check

all: $(TARGET) $(TARGET_ENHANCED) $(TARGET_SIMPLE) $(TARGET_CHECK)

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCE) $(LIBS)

$(TARGET_ENHANCED): $(SOURCE_ENHANCED)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET_ENHANCED) $(SOURCE_ENHANCED) $(LIBS)

$(TARGET_SIMPLE): $(SOURCE_SIMPLE)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET_SIMPLE) $(SOURCE_SIMPLE) $(LIBS)

$(TARGET_CHECK): $(SOURCE_CHECK)
	$(CXX) $(CXXFLAGS) -I/usr/include/seekcamera -I/usr/include/seekframe -o $(TARGET_CHECK) $(SOURCE_CHECK) -lseekcamera -lseekware -lpthread

clean:
	rm -f $(TARGET) $(TARGET_ENHANCED) $(TARGET_SIMPLE) $(TARGET_CHECK)

install: $(TARGET)
	cp $(TARGET) /usr/local/bin/

enhanced: $(TARGET_ENHANCED)

simple: $(TARGET_SIMPLE)

check: $(TARGET_CHECK)

# Examples
example-jet:
	./$(TARGET) -p 0

example-hot:
	./$(TARGET) -p 1

example-adaptive:
	./$(TARGET) -p 4 -a 2

example-minimal:
	./$(TARGET) -t -s -c

help:
	@echo "Available targets:"
	@echo "  all          - Build all thermal viewers"
	@echo "  enhanced     - Build enhanced viewer with SDK pipeline support"
	@echo "  simple       - Build simplified viewer (IMAGE_SEEKVISION + JET only)"
	@echo "  check        - Build camera settings checker"
	@echo "  clean        - Remove built files"
	@echo "  install      - Install to /usr/local/bin"
	@echo "  example-jet  - Run with JET colormap"
	@echo "  example-hot  - Run with HOT colormap"
	@echo "  example-adaptive - Run with VIRIDIS colormap and adaptive AGC"
	@echo "  example-minimal - Run with minimal overlays"
	@echo "  check-settings - Check current camera settings"
	@echo "  help         - Show this help"

check-settings: $(TARGET_CHECK)
	./$(TARGET_CHECK)
