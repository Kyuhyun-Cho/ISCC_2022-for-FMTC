# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/foscar/ISCC_2022/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/foscar/ISCC_2022/build

# Include any dependencies generated for this target.
include gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/depend.make

# Include the progress variables for this target.
include gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/progress.make

# Include the compile flags for this target's objects.
include gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/flags.make

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/flags.make
gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o: /home/foscar/ISCC_2022/src/gps_team/gps/ublox/ublox_gps/src/gps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/foscar/ISCC_2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o"
	cd /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ublox_gps.dir/src/gps.cpp.o -c /home/foscar/ISCC_2022/src/gps_team/gps/ublox/ublox_gps/src/gps.cpp

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ublox_gps.dir/src/gps.cpp.i"
	cd /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/foscar/ISCC_2022/src/gps_team/gps/ublox/ublox_gps/src/gps.cpp > CMakeFiles/ublox_gps.dir/src/gps.cpp.i

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ublox_gps.dir/src/gps.cpp.s"
	cd /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/foscar/ISCC_2022/src/gps_team/gps/ublox/ublox_gps/src/gps.cpp -o CMakeFiles/ublox_gps.dir/src/gps.cpp.s

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.requires:

.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.requires

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.provides: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.requires
	$(MAKE) -f gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/build.make gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.provides.build
.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.provides

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.provides.build: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o


# Object files for target ublox_gps
ublox_gps_OBJECTS = \
"CMakeFiles/ublox_gps.dir/src/gps.cpp.o"

# External object files for target ublox_gps
ublox_gps_EXTERNAL_OBJECTS =

/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/build.make
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /home/foscar/ISCC_2022/devel/lib/libublox_msgs.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/libroscpp.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/librosconsole.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/librostime.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /opt/ros/melodic/lib/libcpp_common.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/foscar/ISCC_2022/devel/lib/libublox_gps.so: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/foscar/ISCC_2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/foscar/ISCC_2022/devel/lib/libublox_gps.so"
	cd /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ublox_gps.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/build: /home/foscar/ISCC_2022/devel/lib/libublox_gps.so

.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/build

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/requires: gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/src/gps.cpp.o.requires

.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/requires

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/clean:
	cd /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps && $(CMAKE_COMMAND) -P CMakeFiles/ublox_gps.dir/cmake_clean.cmake
.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/clean

gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/depend:
	cd /home/foscar/ISCC_2022/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/foscar/ISCC_2022/src /home/foscar/ISCC_2022/src/gps_team/gps/ublox/ublox_gps /home/foscar/ISCC_2022/build /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps /home/foscar/ISCC_2022/build/gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gps_team/gps/ublox/ublox_gps/CMakeFiles/ublox_gps.dir/depend

