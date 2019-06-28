# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/margot/sai-2/apps/Sigma7Applications

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/margot/sai-2/apps/Sigma7Applications/build

# Include any dependencies generated for this target.
include 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/depend.make

# Include the progress variables for this target.
include 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/progress.make

# Include the compile flags for this target's objects.
include 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/flags.make

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/flags.make
04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o: ../04-HapticDeviceHumanInteraction/simviz.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/margot/sai-2/apps/Sigma7Applications/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o"
	cd /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/simviz04.dir/simviz.cpp.o -c /home/margot/sai-2/apps/Sigma7Applications/04-HapticDeviceHumanInteraction/simviz.cpp

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz04.dir/simviz.cpp.i"
	cd /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/margot/sai-2/apps/Sigma7Applications/04-HapticDeviceHumanInteraction/simviz.cpp > CMakeFiles/simviz04.dir/simviz.cpp.i

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz04.dir/simviz.cpp.s"
	cd /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/margot/sai-2/apps/Sigma7Applications/04-HapticDeviceHumanInteraction/simviz.cpp -o CMakeFiles/simviz04.dir/simviz.cpp.s

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.requires:
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.requires

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.provides: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.requires
	$(MAKE) -f 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/build.make 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.provides.build
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.provides

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.provides.build: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o

# Object files for target simviz04
simviz04_OBJECTS = \
"CMakeFiles/simviz04.dir/simviz.cpp.o"

# External object files for target simviz04
simviz04_EXTERNAL_OBJECTS =

../bin/04-HapticDeviceHumanInteraction/simviz04: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o
../bin/04-HapticDeviceHumanInteraction/simviz04: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/build.make
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-common/build/libsai2-common.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/Documents/chai3d-3.2.0/build/libchai3d.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-simulation/build/libsai2-simulation.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-model/build/libsai2-model.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-model/rbdl/build/librbdl.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-graphics/build/libsai2-graphics.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/Documents/chai3d-3.2.0/build/libchai3d.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-primitives/build/libsai2-primitives.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-primitives/../external/ReflexxesTypeIV/Linux/x64/release/lib/shared/libReflexxesTypeIV.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-common/build/libsai2-common.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/Documents/chai3d-3.2.0/build/libchai3d.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-simulation/build/libsai2-simulation.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-model/build/libsai2-model.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-model/rbdl/build/librbdl.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-graphics/build/libsai2-graphics.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/Documents/chai3d-3.2.0/build/libchai3d.a
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /home/margot/sai-2/core/sai2-primitives/../external/ReflexxesTypeIV/Linux/x64/release/lib/shared/libReflexxesTypeIV.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/04-HapticDeviceHumanInteraction/simviz04: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/04-HapticDeviceHumanInteraction/simviz04: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/04-HapticDeviceHumanInteraction/simviz04"
	cd /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz04.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/build: ../bin/04-HapticDeviceHumanInteraction/simviz04
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/build

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/requires: 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/simviz.cpp.o.requires
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/requires

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/clean:
	cd /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction && $(CMAKE_COMMAND) -P CMakeFiles/simviz04.dir/cmake_clean.cmake
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/clean

04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/depend:
	cd /home/margot/sai-2/apps/Sigma7Applications/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/margot/sai-2/apps/Sigma7Applications /home/margot/sai-2/apps/Sigma7Applications/04-HapticDeviceHumanInteraction /home/margot/sai-2/apps/Sigma7Applications/build /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction /home/margot/sai-2/apps/Sigma7Applications/build/04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : 04-HapticDeviceHumanInteraction/CMakeFiles/simviz04.dir/depend

