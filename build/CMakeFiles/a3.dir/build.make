# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Alenta/Desktop/testing/Fluid-Simulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Alenta/Desktop/testing/Fluid-Simulation/build

# Include any dependencies generated for this target.
include CMakeFiles/a3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/a3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a3.dir/flags.make

CMakeFiles/a3.dir/src/main.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/a3.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/main.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/main.cpp

CMakeFiles/a3.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/main.cpp > CMakeFiles/a3.dir/src/main.cpp.i

CMakeFiles/a3.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/main.cpp -o CMakeFiles/a3.dir/src/main.cpp.s

CMakeFiles/a3.dir/src/starter3_util.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/starter3_util.cpp.o: ../src/starter3_util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/a3.dir/src/starter3_util.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/starter3_util.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/starter3_util.cpp

CMakeFiles/a3.dir/src/starter3_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/starter3_util.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/starter3_util.cpp > CMakeFiles/a3.dir/src/starter3_util.cpp.i

CMakeFiles/a3.dir/src/starter3_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/starter3_util.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/starter3_util.cpp -o CMakeFiles/a3.dir/src/starter3_util.cpp.s

CMakeFiles/a3.dir/src/camera.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/a3.dir/src/camera.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/camera.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/camera.cpp

CMakeFiles/a3.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/camera.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/camera.cpp > CMakeFiles/a3.dir/src/camera.cpp.i

CMakeFiles/a3.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/camera.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/camera.cpp -o CMakeFiles/a3.dir/src/camera.cpp.s

CMakeFiles/a3.dir/src/vertexrecorder.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/vertexrecorder.cpp.o: ../src/vertexrecorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/a3.dir/src/vertexrecorder.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/vertexrecorder.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/vertexrecorder.cpp

CMakeFiles/a3.dir/src/vertexrecorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/vertexrecorder.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/vertexrecorder.cpp > CMakeFiles/a3.dir/src/vertexrecorder.cpp.i

CMakeFiles/a3.dir/src/vertexrecorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/vertexrecorder.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/vertexrecorder.cpp -o CMakeFiles/a3.dir/src/vertexrecorder.cpp.s

CMakeFiles/a3.dir/src/clothsystem.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/clothsystem.cpp.o: ../src/clothsystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/a3.dir/src/clothsystem.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/clothsystem.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/clothsystem.cpp

CMakeFiles/a3.dir/src/clothsystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/clothsystem.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/clothsystem.cpp > CMakeFiles/a3.dir/src/clothsystem.cpp.i

CMakeFiles/a3.dir/src/clothsystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/clothsystem.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/clothsystem.cpp -o CMakeFiles/a3.dir/src/clothsystem.cpp.s

CMakeFiles/a3.dir/src/timestepper.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/timestepper.cpp.o: ../src/timestepper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/a3.dir/src/timestepper.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/timestepper.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/timestepper.cpp

CMakeFiles/a3.dir/src/timestepper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/timestepper.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/timestepper.cpp > CMakeFiles/a3.dir/src/timestepper.cpp.i

CMakeFiles/a3.dir/src/timestepper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/timestepper.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/timestepper.cpp -o CMakeFiles/a3.dir/src/timestepper.cpp.s

CMakeFiles/a3.dir/src/particlesystem.cpp.o: CMakeFiles/a3.dir/flags.make
CMakeFiles/a3.dir/src/particlesystem.cpp.o: ../src/particlesystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/a3.dir/src/particlesystem.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3.dir/src/particlesystem.cpp.o -c /Users/Alenta/Desktop/testing/Fluid-Simulation/src/particlesystem.cpp

CMakeFiles/a3.dir/src/particlesystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3.dir/src/particlesystem.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Alenta/Desktop/testing/Fluid-Simulation/src/particlesystem.cpp > CMakeFiles/a3.dir/src/particlesystem.cpp.i

CMakeFiles/a3.dir/src/particlesystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3.dir/src/particlesystem.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Alenta/Desktop/testing/Fluid-Simulation/src/particlesystem.cpp -o CMakeFiles/a3.dir/src/particlesystem.cpp.s

# Object files for target a3
a3_OBJECTS = \
"CMakeFiles/a3.dir/src/main.cpp.o" \
"CMakeFiles/a3.dir/src/starter3_util.cpp.o" \
"CMakeFiles/a3.dir/src/camera.cpp.o" \
"CMakeFiles/a3.dir/src/vertexrecorder.cpp.o" \
"CMakeFiles/a3.dir/src/clothsystem.cpp.o" \
"CMakeFiles/a3.dir/src/timestepper.cpp.o" \
"CMakeFiles/a3.dir/src/particlesystem.cpp.o"

# External object files for target a3
a3_EXTERNAL_OBJECTS =

a3: CMakeFiles/a3.dir/src/main.cpp.o
a3: CMakeFiles/a3.dir/src/starter3_util.cpp.o
a3: CMakeFiles/a3.dir/src/camera.cpp.o
a3: CMakeFiles/a3.dir/src/vertexrecorder.cpp.o
a3: CMakeFiles/a3.dir/src/clothsystem.cpp.o
a3: CMakeFiles/a3.dir/src/timestepper.cpp.o
a3: CMakeFiles/a3.dir/src/particlesystem.cpp.o
a3: CMakeFiles/a3.dir/build.make
a3: 3rd_party/glfw/src/libglfw3.a
a3: vecmath/libvecmath.a
a3: CMakeFiles/a3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable a3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a3.dir/build: a3

.PHONY : CMakeFiles/a3.dir/build

CMakeFiles/a3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a3.dir/clean

CMakeFiles/a3.dir/depend:
	cd /Users/Alenta/Desktop/testing/Fluid-Simulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Alenta/Desktop/testing/Fluid-Simulation /Users/Alenta/Desktop/testing/Fluid-Simulation /Users/Alenta/Desktop/testing/Fluid-Simulation/build /Users/Alenta/Desktop/testing/Fluid-Simulation/build /Users/Alenta/Desktop/testing/Fluid-Simulation/build/CMakeFiles/a3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a3.dir/depend

