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
include glfw/tests/CMakeFiles/sharing.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include glfw/tests/CMakeFiles/sharing.dir/compiler_depend.make

# Include the progress variables for this target.
include glfw/tests/CMakeFiles/sharing.dir/progress.make

# Include the compile flags for this target's objects.
include glfw/tests/CMakeFiles/sharing.dir/flags.make

glfw/tests/CMakeFiles/sharing.dir/sharing.c.o: glfw/tests/CMakeFiles/sharing.dir/flags.make
glfw/tests/CMakeFiles/sharing.dir/sharing.c.o: ../glfw/tests/sharing.c
glfw/tests/CMakeFiles/sharing.dir/sharing.c.o: glfw/tests/CMakeFiles/sharing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object glfw/tests/CMakeFiles/sharing.dir/sharing.c.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT glfw/tests/CMakeFiles/sharing.dir/sharing.c.o -MF CMakeFiles/sharing.dir/sharing.c.o.d -o CMakeFiles/sharing.dir/sharing.c.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/tests/sharing.c"

glfw/tests/CMakeFiles/sharing.dir/sharing.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sharing.dir/sharing.c.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/tests/sharing.c" > CMakeFiles/sharing.dir/sharing.c.i

glfw/tests/CMakeFiles/sharing.dir/sharing.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sharing.dir/sharing.c.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/tests/sharing.c" -o CMakeFiles/sharing.dir/sharing.c.s

glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o: glfw/tests/CMakeFiles/sharing.dir/flags.make
glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o: ../glfw/deps/glad.c
glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o: glfw/tests/CMakeFiles/sharing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building C object glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o -MF CMakeFiles/sharing.dir/__/deps/glad.c.o.d -o CMakeFiles/sharing.dir/__/deps/glad.c.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c"

glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sharing.dir/__/deps/glad.c.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c" > CMakeFiles/sharing.dir/__/deps/glad.c.i

glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sharing.dir/__/deps/glad.c.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/deps/glad.c" -o CMakeFiles/sharing.dir/__/deps/glad.c.s

# Object files for target sharing
sharing_OBJECTS = \
"CMakeFiles/sharing.dir/sharing.c.o" \
"CMakeFiles/sharing.dir/__/deps/glad.c.o"

# External object files for target sharing
sharing_EXTERNAL_OBJECTS =

glfw/tests/sharing: glfw/tests/CMakeFiles/sharing.dir/sharing.c.o
glfw/tests/sharing: glfw/tests/CMakeFiles/sharing.dir/__/deps/glad.c.o
glfw/tests/sharing: glfw/tests/CMakeFiles/sharing.dir/build.make
glfw/tests/sharing: glfw/src/libglfw3.a
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/librt.a
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libm.so
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libX11.so
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libXrandr.so
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libXinerama.so
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
glfw/tests/sharing: /usr/lib/x86_64-linux-gnu/libXcursor.so
glfw/tests/sharing: glfw/tests/CMakeFiles/sharing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable sharing"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sharing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
glfw/tests/CMakeFiles/sharing.dir/build: glfw/tests/sharing
.PHONY : glfw/tests/CMakeFiles/sharing.dir/build

glfw/tests/CMakeFiles/sharing.dir/clean:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" && $(CMAKE_COMMAND) -P CMakeFiles/sharing.dir/cmake_clean.cmake
.PHONY : glfw/tests/CMakeFiles/sharing.dir/clean

glfw/tests/CMakeFiles/sharing.dir/depend:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/media/sf_Learn/Computer Graphics/project1/starter1" "/media/sf_Learn/Computer Graphics/project1/starter1/glfw/tests" "/media/sf_Learn/Computer Graphics/project1/starter1/build" "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests" "/media/sf_Learn/Computer Graphics/project1/starter1/build/glfw/tests/CMakeFiles/sharing.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : glfw/tests/CMakeFiles/sharing.dir/depend

