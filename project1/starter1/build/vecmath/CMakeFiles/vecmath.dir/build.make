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
include vecmath/CMakeFiles/vecmath.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include vecmath/CMakeFiles/vecmath.dir/compiler_depend.make

# Include the progress variables for this target.
include vecmath/CMakeFiles/vecmath.dir/progress.make

# Include the compile flags for this target's objects.
include vecmath/CMakeFiles/vecmath.dir/flags.make

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o: ../vecmath/Matrix2f.cpp
vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o -MF CMakeFiles/vecmath.dir/Matrix2f.cpp.o.d -o CMakeFiles/vecmath.dir/Matrix2f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix2f.cpp"

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix2f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix2f.cpp" > CMakeFiles/vecmath.dir/Matrix2f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix2f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix2f.cpp" -o CMakeFiles/vecmath.dir/Matrix2f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o: ../vecmath/Matrix3f.cpp
vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o -MF CMakeFiles/vecmath.dir/Matrix3f.cpp.o.d -o CMakeFiles/vecmath.dir/Matrix3f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix3f.cpp"

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix3f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix3f.cpp" > CMakeFiles/vecmath.dir/Matrix3f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix3f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix3f.cpp" -o CMakeFiles/vecmath.dir/Matrix3f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o: ../vecmath/Matrix4f.cpp
vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o -MF CMakeFiles/vecmath.dir/Matrix4f.cpp.o.d -o CMakeFiles/vecmath.dir/Matrix4f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix4f.cpp"

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix4f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix4f.cpp" > CMakeFiles/vecmath.dir/Matrix4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix4f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Matrix4f.cpp" -o CMakeFiles/vecmath.dir/Matrix4f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o: ../vecmath/Quat4f.cpp
vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o -MF CMakeFiles/vecmath.dir/Quat4f.cpp.o.d -o CMakeFiles/vecmath.dir/Quat4f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Quat4f.cpp"

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Quat4f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Quat4f.cpp" > CMakeFiles/vecmath.dir/Quat4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Quat4f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Quat4f.cpp" -o CMakeFiles/vecmath.dir/Quat4f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o: ../vecmath/Vector2f.cpp
vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o -MF CMakeFiles/vecmath.dir/Vector2f.cpp.o.d -o CMakeFiles/vecmath.dir/Vector2f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector2f.cpp"

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector2f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector2f.cpp" > CMakeFiles/vecmath.dir/Vector2f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector2f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector2f.cpp" -o CMakeFiles/vecmath.dir/Vector2f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o: ../vecmath/Vector3f.cpp
vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o -MF CMakeFiles/vecmath.dir/Vector3f.cpp.o.d -o CMakeFiles/vecmath.dir/Vector3f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector3f.cpp"

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector3f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector3f.cpp" > CMakeFiles/vecmath.dir/Vector3f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector3f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector3f.cpp" -o CMakeFiles/vecmath.dir/Vector3f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o: ../vecmath/Vector4f.cpp
vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o -MF CMakeFiles/vecmath.dir/Vector4f.cpp.o.d -o CMakeFiles/vecmath.dir/Vector4f.cpp.o -c "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector4f.cpp"

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector4f.cpp.i"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector4f.cpp" > CMakeFiles/vecmath.dir/Vector4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector4f.cpp.s"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath/Vector4f.cpp" -o CMakeFiles/vecmath.dir/Vector4f.cpp.s

# Object files for target vecmath
vecmath_OBJECTS = \
"CMakeFiles/vecmath.dir/Matrix2f.cpp.o" \
"CMakeFiles/vecmath.dir/Matrix3f.cpp.o" \
"CMakeFiles/vecmath.dir/Matrix4f.cpp.o" \
"CMakeFiles/vecmath.dir/Quat4f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector2f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector3f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector4f.cpp.o"

# External object files for target vecmath
vecmath_EXTERNAL_OBJECTS =

vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/build.make
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/media/sf_Learn/Computer Graphics/project1/starter1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libvecmath.a"
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && $(CMAKE_COMMAND) -P CMakeFiles/vecmath.dir/cmake_clean_target.cmake
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vecmath.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vecmath/CMakeFiles/vecmath.dir/build: vecmath/libvecmath.a
.PHONY : vecmath/CMakeFiles/vecmath.dir/build

vecmath/CMakeFiles/vecmath.dir/clean:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" && $(CMAKE_COMMAND) -P CMakeFiles/vecmath.dir/cmake_clean.cmake
.PHONY : vecmath/CMakeFiles/vecmath.dir/clean

vecmath/CMakeFiles/vecmath.dir/depend:
	cd "/media/sf_Learn/Computer Graphics/project1/starter1/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/media/sf_Learn/Computer Graphics/project1/starter1" "/media/sf_Learn/Computer Graphics/project1/starter1/vecmath" "/media/sf_Learn/Computer Graphics/project1/starter1/build" "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath" "/media/sf_Learn/Computer Graphics/project1/starter1/build/vecmath/CMakeFiles/vecmath.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : vecmath/CMakeFiles/vecmath.dir/depend

