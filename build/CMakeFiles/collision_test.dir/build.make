# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/wsl/proj/skyvortex_mujoco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wsl/proj/skyvortex_mujoco/build

# Include any dependencies generated for this target.
include CMakeFiles/collision_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/collision_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/collision_test.dir/flags.make

CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o: CMakeFiles/collision_test.dir/flags.make
CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o: ../src/collision_checker_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wsl/proj/skyvortex_mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o -c /home/wsl/proj/skyvortex_mujoco/src/collision_checker_test.cpp

CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wsl/proj/skyvortex_mujoco/src/collision_checker_test.cpp > CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.i

CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wsl/proj/skyvortex_mujoco/src/collision_checker_test.cpp -o CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.s

# Object files for target collision_test
collision_test_OBJECTS = \
"CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o"

# External object files for target collision_test
collision_test_EXTERNAL_OBJECTS =

collision_test: CMakeFiles/collision_test.dir/src/collision_checker_test.cpp.o
collision_test: CMakeFiles/collision_test.dir/build.make
collision_test: libmujoco_client.a
collision_test: /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
collision_test: /usr/lib/x86_64-linux-gnu/libGLX.so
collision_test: /usr/lib/x86_64-linux-gnu/libOpenGL.so
collision_test: CMakeFiles/collision_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wsl/proj/skyvortex_mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable collision_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/collision_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/collision_test.dir/build: collision_test

.PHONY : CMakeFiles/collision_test.dir/build

CMakeFiles/collision_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/collision_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/collision_test.dir/clean

CMakeFiles/collision_test.dir/depend:
	cd /home/wsl/proj/skyvortex_mujoco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wsl/proj/skyvortex_mujoco /home/wsl/proj/skyvortex_mujoco /home/wsl/proj/skyvortex_mujoco/build /home/wsl/proj/skyvortex_mujoco/build /home/wsl/proj/skyvortex_mujoco/build/CMakeFiles/collision_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/collision_test.dir/depend

