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
CMAKE_SOURCE_DIR = "/media/sf_Learn/Computer Graphics/project1/starter1"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/media/sf_Learn/Computer Graphics/project1/starter1/build"

# Include any dependencies generated for this target.
include glfw/examples/CMakeFiles/simple.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include glfw/examples/CMakeFiles/simple.dir/compiler_depend.make

# Include the progress variables for this target.
include glfw/examples/CMakeFiles/simple.dir/progress.make

# Include the compile flags for this target's objects.
include glfw/examples/CMakeFiles/simple.dir/flags.make

glfw/examples/CMakeFiles/simple.dir/simple.c.o: glfw/examples/CMakeFiles/simple.dir/flags.make
glfw/examples/CMakeFiles/simple.dir/simple.c.o: ../glfw/examples/simple.c
glfw/examples/CMakeFiles/simple.dir/simple.c.o: glfw/examples/CMakeFiles/simple.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object glfw/examples/CMakeFiles/simple.dir/simple.c.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT glfw/examples/CMakeFiles/simple.dir/simple.c.o -MF CMakeFiles/simple.dir/simple.c.o.d -o CMakeFiles/simple.dir/simple.c.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/examples/simple.c"

glfw/examples/CMakeFiles/simple.dir/simple.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simple.dir/simple.c.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/examples/simple.c" > CMakeFiles/simple.dir/simple.c.i

glfw/examples/CMakeFiles/simple.dir/simple.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simple.dir/simple.c.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/examples/simple.c" -o CMakeFiles/simple.dir/simple.c.s

glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o: glfw/examples/CMakeFiles/simple.dir/flags.make
glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o: ../glfw/deps/glad.c
glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o: glfw/examples/CMakeFiles/simple.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building C object glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o -MF CMakeFiles/simple.dir/__/deps/glad.c.o.d -o CMakeFiles/simple.dir/__/deps/glad.c.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c"

glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simple.dir/__/deps/glad.c.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c" > CMakeFiles/simple.dir/__/deps/glad.c.i

glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simple.dir/__/deps/glad.c.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c" -o CMakeFiles/simple.dir/__/deps/glad.c.s

# Object files for target simple
simple_OBJECTS = \
"CMakeFiles/simple.dir/simple.c.o" \
"CMakeFiles/simple.dir/__/deps/glad.c.o"

# External object files for target simple
simple_EXTERNAL_OBJECTS =

glfw/examples/simple: glfw/examples/CMakeFiles/simple.dir/simple.c.o
glfw/examples/simple: glfw/examples/CMakeFiles/simple.dir/__/deps/glad.c.o
glfw/examples/simple: glfw/examples/CMakeFiles/simple.dir/build.make
glfw/examples/simple: glfw/src/libglfw3.a
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/librt.a
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libm.so
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libX11.so
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libXrandr.so
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libXinerama.so
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
glfw/examples/simple: /usr/lib/x86_64-linux-gnu/libXcursor.so
glfw/examples/simple: glfw/examples/CMakeFiles/simple.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable simple"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
glfw/examples/CMakeFiles/simple.dir/build: glfw/examples/simple
.PHONY : glfw/examples/CMakeFiles/simple.dir/build

glfw/examples/CMakeFiles/simple.dir/clean:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" && $(CMAKE_COMMAND) -P CMakeFiles/simple.dir/cmake_clean.cmake
.PHONY : glfw/examples/CMakeFiles/simple.dir/clean

glfw/examples/CMakeFiles/simple.dir/depend:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/media/sf_Learn/Computer Graphics/project1/starter1" "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/examples" "/media/sf_Learn/Computer Graphics/project1/starter1/build" "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples" "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/examples/CMakeFiles/simple.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : glfw/examples/CMakeFiles/simple.dir/depend

