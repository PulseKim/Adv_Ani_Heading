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
CMAKE_SOURCE_DIR = "/home/sonic/Projects/Animation Project"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sonic/Projects/Animation Project/build"

# Include any dependencies generated for this target.
include CMakeFiles/run.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run.dir/flags.make

CMakeFiles/run.dir/Controller.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/Controller.cpp.o: ../Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sonic/Projects/Animation Project/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run.dir/Controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/Controller.cpp.o -c "/home/sonic/Projects/Animation Project/Controller.cpp"

CMakeFiles/run.dir/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/Controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sonic/Projects/Animation Project/Controller.cpp" > CMakeFiles/run.dir/Controller.cpp.i

CMakeFiles/run.dir/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/Controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sonic/Projects/Animation Project/Controller.cpp" -o CMakeFiles/run.dir/Controller.cpp.s

CMakeFiles/run.dir/Controller.cpp.o.requires:

.PHONY : CMakeFiles/run.dir/Controller.cpp.o.requires

CMakeFiles/run.dir/Controller.cpp.o.provides: CMakeFiles/run.dir/Controller.cpp.o.requires
	$(MAKE) -f CMakeFiles/run.dir/build.make CMakeFiles/run.dir/Controller.cpp.o.provides.build
.PHONY : CMakeFiles/run.dir/Controller.cpp.o.provides

CMakeFiles/run.dir/Controller.cpp.o.provides.build: CMakeFiles/run.dir/Controller.cpp.o


CMakeFiles/run.dir/MyWindow.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/MyWindow.cpp.o: ../MyWindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sonic/Projects/Animation Project/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/run.dir/MyWindow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/MyWindow.cpp.o -c "/home/sonic/Projects/Animation Project/MyWindow.cpp"

CMakeFiles/run.dir/MyWindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/MyWindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sonic/Projects/Animation Project/MyWindow.cpp" > CMakeFiles/run.dir/MyWindow.cpp.i

CMakeFiles/run.dir/MyWindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/MyWindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sonic/Projects/Animation Project/MyWindow.cpp" -o CMakeFiles/run.dir/MyWindow.cpp.s

CMakeFiles/run.dir/MyWindow.cpp.o.requires:

.PHONY : CMakeFiles/run.dir/MyWindow.cpp.o.requires

CMakeFiles/run.dir/MyWindow.cpp.o.provides: CMakeFiles/run.dir/MyWindow.cpp.o.requires
	$(MAKE) -f CMakeFiles/run.dir/build.make CMakeFiles/run.dir/MyWindow.cpp.o.provides.build
.PHONY : CMakeFiles/run.dir/MyWindow.cpp.o.provides

CMakeFiles/run.dir/MyWindow.cpp.o.provides.build: CMakeFiles/run.dir/MyWindow.cpp.o


CMakeFiles/run.dir/SkelGen.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/SkelGen.cpp.o: ../SkelGen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sonic/Projects/Animation Project/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/run.dir/SkelGen.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/SkelGen.cpp.o -c "/home/sonic/Projects/Animation Project/SkelGen.cpp"

CMakeFiles/run.dir/SkelGen.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/SkelGen.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sonic/Projects/Animation Project/SkelGen.cpp" > CMakeFiles/run.dir/SkelGen.cpp.i

CMakeFiles/run.dir/SkelGen.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/SkelGen.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sonic/Projects/Animation Project/SkelGen.cpp" -o CMakeFiles/run.dir/SkelGen.cpp.s

CMakeFiles/run.dir/SkelGen.cpp.o.requires:

.PHONY : CMakeFiles/run.dir/SkelGen.cpp.o.requires

CMakeFiles/run.dir/SkelGen.cpp.o.provides: CMakeFiles/run.dir/SkelGen.cpp.o.requires
	$(MAKE) -f CMakeFiles/run.dir/build.make CMakeFiles/run.dir/SkelGen.cpp.o.provides.build
.PHONY : CMakeFiles/run.dir/SkelGen.cpp.o.provides

CMakeFiles/run.dir/SkelGen.cpp.o.provides.build: CMakeFiles/run.dir/SkelGen.cpp.o


CMakeFiles/run.dir/main.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sonic/Projects/Animation Project/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/run.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/main.cpp.o -c "/home/sonic/Projects/Animation Project/main.cpp"

CMakeFiles/run.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sonic/Projects/Animation Project/main.cpp" > CMakeFiles/run.dir/main.cpp.i

CMakeFiles/run.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sonic/Projects/Animation Project/main.cpp" -o CMakeFiles/run.dir/main.cpp.s

CMakeFiles/run.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/run.dir/main.cpp.o.requires

CMakeFiles/run.dir/main.cpp.o.provides: CMakeFiles/run.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/run.dir/build.make CMakeFiles/run.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/run.dir/main.cpp.o.provides

CMakeFiles/run.dir/main.cpp.o.provides.build: CMakeFiles/run.dir/main.cpp.o


# Object files for target run
run_OBJECTS = \
"CMakeFiles/run.dir/Controller.cpp.o" \
"CMakeFiles/run.dir/MyWindow.cpp.o" \
"CMakeFiles/run.dir/SkelGen.cpp.o" \
"CMakeFiles/run.dir/main.cpp.o"

# External object files for target run
run_EXTERNAL_OBJECTS =

run: CMakeFiles/run.dir/Controller.cpp.o
run: CMakeFiles/run.dir/MyWindow.cpp.o
run: CMakeFiles/run.dir/SkelGen.cpp.o
run: CMakeFiles/run.dir/main.cpp.o
run: CMakeFiles/run.dir/build.make
run: /usr/local/lib/libdart-gui.so.6.8.2
run: /usr/local/lib/libdart-optimizer-ipopt.so.6.8.2
run: /usr/local/lib/libdart-collision-bullet.so.6.8.2
run: /usr/local/lib/libdart-collision-ode.so.6.8.2
run: /usr/local/lib/libdart-utils.so.6.8.2
run: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
run: /usr/lib/x86_64-linux-gnu/libglut.so
run: /usr/lib/x86_64-linux-gnu/libXmu.so
run: /usr/lib/x86_64-linux-gnu/libXi.so
run: /usr/lib/x86_64-linux-gnu/libGLU.so
run: /usr/local/lib/libdart-external-lodepng.so.6.8.2
run: /usr/local/lib/libdart-external-imgui.so.6.8.2
run: /usr/lib/x86_64-linux-gnu/libGL.so
run: /usr/lib/libipopt.so
run: /usr/lib/x86_64-linux-gnu/libBulletDynamics.so
run: /usr/lib/x86_64-linux-gnu/libBulletCollision.so
run: /usr/lib/x86_64-linux-gnu/libLinearMath.so
run: /usr/lib/x86_64-linux-gnu/libBulletSoftBody.so
run: /usr/local/lib/libdart.so.6.8.2
run: /usr/local/lib/libdart-external-odelcpsolver.so.6.8.2
run: /usr/lib/x86_64-linux-gnu/libccd.so
run: /usr/lib/x86_64-linux-gnu/libfcl.so
run: /usr/lib/x86_64-linux-gnu/libassimp.so
run: /usr/local/lib/libboost_filesystem.so
run: /usr/local/lib/libboost_system.so
run: /usr/lib/liboctomap.so
run: /usr/lib/liboctomath.so
run: /usr/local/lib/libboost_regex.so
run: /usr/lib/x86_64-linux-gnu/libode.so
run: CMakeFiles/run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sonic/Projects/Animation Project/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable run"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run.dir/build: run

.PHONY : CMakeFiles/run.dir/build

CMakeFiles/run.dir/requires: CMakeFiles/run.dir/Controller.cpp.o.requires
CMakeFiles/run.dir/requires: CMakeFiles/run.dir/MyWindow.cpp.o.requires
CMakeFiles/run.dir/requires: CMakeFiles/run.dir/SkelGen.cpp.o.requires
CMakeFiles/run.dir/requires: CMakeFiles/run.dir/main.cpp.o.requires

.PHONY : CMakeFiles/run.dir/requires

CMakeFiles/run.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run.dir/clean

CMakeFiles/run.dir/depend:
	cd "/home/sonic/Projects/Animation Project/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sonic/Projects/Animation Project" "/home/sonic/Projects/Animation Project" "/home/sonic/Projects/Animation Project/build" "/home/sonic/Projects/Animation Project/build" "/home/sonic/Projects/Animation Project/build/CMakeFiles/run.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/run.dir/depend

