# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/why/Desktop/CGlab1/FDU_CG/project1/starter1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy

# Include any dependencies generated for this target.
include CMakeFiles/a1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/a1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/a1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a1.dir/flags.make

CMakeFiles/a1.dir/glew/src/glew.c.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/glew/src/glew.c.o: ../glew/src/glew.c
CMakeFiles/a1.dir/glew/src/glew.c.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/a1.dir/glew/src/glew.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/a1.dir/glew/src/glew.c.o -MF CMakeFiles/a1.dir/glew/src/glew.c.o.d -o CMakeFiles/a1.dir/glew/src/glew.c.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/glew/src/glew.c

CMakeFiles/a1.dir/glew/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/a1.dir/glew/src/glew.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/glew/src/glew.c > CMakeFiles/a1.dir/glew/src/glew.c.i

CMakeFiles/a1.dir/glew/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/a1.dir/glew/src/glew.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/glew/src/glew.c -o CMakeFiles/a1.dir/glew/src/glew.c.s

CMakeFiles/a1.dir/src/main.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/a1.dir/src/main.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/a1.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/main.cpp.o -MF CMakeFiles/a1.dir/src/main.cpp.o.d -o CMakeFiles/a1.dir/src/main.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/main.cpp

CMakeFiles/a1.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/main.cpp > CMakeFiles/a1.dir/src/main.cpp.i

CMakeFiles/a1.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/main.cpp -o CMakeFiles/a1.dir/src/main.cpp.s

CMakeFiles/a1.dir/src/camera.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/camera.cpp.o: ../src/camera.cpp
CMakeFiles/a1.dir/src/camera.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/a1.dir/src/camera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/camera.cpp.o -MF CMakeFiles/a1.dir/src/camera.cpp.o.d -o CMakeFiles/a1.dir/src/camera.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/camera.cpp

CMakeFiles/a1.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/camera.cpp > CMakeFiles/a1.dir/src/camera.cpp.i

CMakeFiles/a1.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/camera.cpp -o CMakeFiles/a1.dir/src/camera.cpp.s

CMakeFiles/a1.dir/src/curve.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/curve.cpp.o: ../src/curve.cpp
CMakeFiles/a1.dir/src/curve.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/a1.dir/src/curve.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/curve.cpp.o -MF CMakeFiles/a1.dir/src/curve.cpp.o.d -o CMakeFiles/a1.dir/src/curve.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/curve.cpp

CMakeFiles/a1.dir/src/curve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/curve.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/curve.cpp > CMakeFiles/a1.dir/src/curve.cpp.i

CMakeFiles/a1.dir/src/curve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/curve.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/curve.cpp -o CMakeFiles/a1.dir/src/curve.cpp.s

CMakeFiles/a1.dir/src/parse.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/parse.cpp.o: ../src/parse.cpp
CMakeFiles/a1.dir/src/parse.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/a1.dir/src/parse.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/parse.cpp.o -MF CMakeFiles/a1.dir/src/parse.cpp.o.d -o CMakeFiles/a1.dir/src/parse.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/parse.cpp

CMakeFiles/a1.dir/src/parse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/parse.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/parse.cpp > CMakeFiles/a1.dir/src/parse.cpp.i

CMakeFiles/a1.dir/src/parse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/parse.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/parse.cpp -o CMakeFiles/a1.dir/src/parse.cpp.s

CMakeFiles/a1.dir/src/starter1_util.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/starter1_util.cpp.o: ../src/starter1_util.cpp
CMakeFiles/a1.dir/src/starter1_util.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/a1.dir/src/starter1_util.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/starter1_util.cpp.o -MF CMakeFiles/a1.dir/src/starter1_util.cpp.o.d -o CMakeFiles/a1.dir/src/starter1_util.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/starter1_util.cpp

CMakeFiles/a1.dir/src/starter1_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/starter1_util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/starter1_util.cpp > CMakeFiles/a1.dir/src/starter1_util.cpp.i

CMakeFiles/a1.dir/src/starter1_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/starter1_util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/starter1_util.cpp -o CMakeFiles/a1.dir/src/starter1_util.cpp.s

CMakeFiles/a1.dir/src/surf.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/surf.cpp.o: ../src/surf.cpp
CMakeFiles/a1.dir/src/surf.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/a1.dir/src/surf.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/surf.cpp.o -MF CMakeFiles/a1.dir/src/surf.cpp.o.d -o CMakeFiles/a1.dir/src/surf.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/surf.cpp

CMakeFiles/a1.dir/src/surf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/surf.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/surf.cpp > CMakeFiles/a1.dir/src/surf.cpp.i

CMakeFiles/a1.dir/src/surf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/surf.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/surf.cpp -o CMakeFiles/a1.dir/src/surf.cpp.s

CMakeFiles/a1.dir/src/vertexrecorder.cpp.o: CMakeFiles/a1.dir/flags.make
CMakeFiles/a1.dir/src/vertexrecorder.cpp.o: ../src/vertexrecorder.cpp
CMakeFiles/a1.dir/src/vertexrecorder.cpp.o: CMakeFiles/a1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/a1.dir/src/vertexrecorder.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1.dir/src/vertexrecorder.cpp.o -MF CMakeFiles/a1.dir/src/vertexrecorder.cpp.o.d -o CMakeFiles/a1.dir/src/vertexrecorder.cpp.o -c /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/vertexrecorder.cpp

CMakeFiles/a1.dir/src/vertexrecorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1.dir/src/vertexrecorder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/vertexrecorder.cpp > CMakeFiles/a1.dir/src/vertexrecorder.cpp.i

CMakeFiles/a1.dir/src/vertexrecorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1.dir/src/vertexrecorder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/src/vertexrecorder.cpp -o CMakeFiles/a1.dir/src/vertexrecorder.cpp.s

# Object files for target a1
a1_OBJECTS = \
"CMakeFiles/a1.dir/glew/src/glew.c.o" \
"CMakeFiles/a1.dir/src/main.cpp.o" \
"CMakeFiles/a1.dir/src/camera.cpp.o" \
"CMakeFiles/a1.dir/src/curve.cpp.o" \
"CMakeFiles/a1.dir/src/parse.cpp.o" \
"CMakeFiles/a1.dir/src/starter1_util.cpp.o" \
"CMakeFiles/a1.dir/src/surf.cpp.o" \
"CMakeFiles/a1.dir/src/vertexrecorder.cpp.o"

# External object files for target a1
a1_EXTERNAL_OBJECTS =

a1: CMakeFiles/a1.dir/glew/src/glew.c.o
a1: CMakeFiles/a1.dir/src/main.cpp.o
a1: CMakeFiles/a1.dir/src/camera.cpp.o
a1: CMakeFiles/a1.dir/src/curve.cpp.o
a1: CMakeFiles/a1.dir/src/parse.cpp.o
a1: CMakeFiles/a1.dir/src/starter1_util.cpp.o
a1: CMakeFiles/a1.dir/src/surf.cpp.o
a1: CMakeFiles/a1.dir/src/vertexrecorder.cpp.o
a1: CMakeFiles/a1.dir/build.make
a1: /usr/lib/aarch64-linux-gnu/libGL.so
a1: glfw/src/libglfw3.a
a1: vecmath/libvecmath.a
a1: /usr/lib/aarch64-linux-gnu/librt.a
a1: /usr/lib/aarch64-linux-gnu/libm.so
a1: /usr/lib/aarch64-linux-gnu/libX11.so
a1: /usr/lib/aarch64-linux-gnu/libXrandr.so
a1: /usr/lib/aarch64-linux-gnu/libXinerama.so
a1: /usr/lib/aarch64-linux-gnu/libXxf86vm.so
a1: /usr/lib/aarch64-linux-gnu/libXcursor.so
a1: CMakeFiles/a1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable a1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a1.dir/build: a1
.PHONY : CMakeFiles/a1.dir/build

CMakeFiles/a1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a1.dir/clean

CMakeFiles/a1.dir/depend:
	cd /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/why/Desktop/CGlab1/FDU_CG/project1/starter1 /home/why/Desktop/CGlab1/FDU_CG/project1/starter1 /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy /home/why/Desktop/CGlab1/FDU_CG/project1/starter1/buildwhy/CMakeFiles/a1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a1.dir/depend
