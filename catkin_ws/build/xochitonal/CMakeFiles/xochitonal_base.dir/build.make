# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/catkin_ws/build

# Include any dependencies generated for this target.
include xochitonal/CMakeFiles/xochitonal_base.dir/depend.make

# Include the progress variables for this target.
include xochitonal/CMakeFiles/xochitonal_base.dir/progress.make

# Include the compile flags for this target's objects.
include xochitonal/CMakeFiles/xochitonal_base.dir/flags.make

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o: xochitonal/CMakeFiles/xochitonal_base.dir/flags.make
xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o: /home/pi/catkin_ws/src/xochitonal/src/xochitonal_base.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o"
	cd /home/pi/catkin_ws/build/xochitonal && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o -c /home/pi/catkin_ws/src/xochitonal/src/xochitonal_base.cpp

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.i"
	cd /home/pi/catkin_ws/build/xochitonal && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/catkin_ws/src/xochitonal/src/xochitonal_base.cpp > CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.i

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.s"
	cd /home/pi/catkin_ws/build/xochitonal && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/catkin_ws/src/xochitonal/src/xochitonal_base.cpp -o CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.s

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.requires:

.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.requires

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.provides: xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.requires
	$(MAKE) -f xochitonal/CMakeFiles/xochitonal_base.dir/build.make xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.provides.build
.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.provides

xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.provides.build: xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o


# Object files for target xochitonal_base
xochitonal_base_OBJECTS = \
"CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o"

# External object files for target xochitonal_base
xochitonal_base_EXTERNAL_OBJECTS =

/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: xochitonal/CMakeFiles/xochitonal_base.dir/build.make
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/libroscpp.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/librosconsole.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/librostime.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /opt/ros/indigo/lib/libcpp_common.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base: xochitonal/CMakeFiles/xochitonal_base.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base"
	cd /home/pi/catkin_ws/build/xochitonal && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xochitonal_base.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
xochitonal/CMakeFiles/xochitonal_base.dir/build: /home/pi/catkin_ws/devel/lib/xochitonal/xochitonal_base

.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/build

xochitonal/CMakeFiles/xochitonal_base.dir/requires: xochitonal/CMakeFiles/xochitonal_base.dir/src/xochitonal_base.cpp.o.requires

.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/requires

xochitonal/CMakeFiles/xochitonal_base.dir/clean:
	cd /home/pi/catkin_ws/build/xochitonal && $(CMAKE_COMMAND) -P CMakeFiles/xochitonal_base.dir/cmake_clean.cmake
.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/clean

xochitonal/CMakeFiles/xochitonal_base.dir/depend:
	cd /home/pi/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/catkin_ws/src /home/pi/catkin_ws/src/xochitonal /home/pi/catkin_ws/build /home/pi/catkin_ws/build/xochitonal /home/pi/catkin_ws/build/xochitonal/CMakeFiles/xochitonal_base.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xochitonal/CMakeFiles/xochitonal_base.dir/depend

