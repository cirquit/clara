# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/rewrite/Documents/github-repos/clara

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rewrite/Documents/github-repos/clara/build-debug

# Include any dependencies generated for this target.
include library/CMakeFiles/clara.dir/depend.make

# Include the progress variables for this target.
include library/CMakeFiles/clara.dir/progress.make

# Include the compile flags for this target's objects.
include library/CMakeFiles/clara.dir/flags.make

# Object files for target clara
clara_OBJECTS =

# External object files for target clara
clara_EXTERNAL_OBJECTS =

library/libclara.a: library/CMakeFiles/clara.dir/build.make
library/libclara.a: library/CMakeFiles/clara.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rewrite/Documents/github-repos/clara/build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking CXX static library libclara.a"
	cd /home/rewrite/Documents/github-repos/clara/build-debug/library && $(CMAKE_COMMAND) -P CMakeFiles/clara.dir/cmake_clean_target.cmake
	cd /home/rewrite/Documents/github-repos/clara/build-debug/library && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clara.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
library/CMakeFiles/clara.dir/build: library/libclara.a

.PHONY : library/CMakeFiles/clara.dir/build

library/CMakeFiles/clara.dir/requires:

.PHONY : library/CMakeFiles/clara.dir/requires

library/CMakeFiles/clara.dir/clean:
	cd /home/rewrite/Documents/github-repos/clara/build-debug/library && $(CMAKE_COMMAND) -P CMakeFiles/clara.dir/cmake_clean.cmake
.PHONY : library/CMakeFiles/clara.dir/clean

library/CMakeFiles/clara.dir/depend:
	cd /home/rewrite/Documents/github-repos/clara/build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rewrite/Documents/github-repos/clara /home/rewrite/Documents/github-repos/clara/library /home/rewrite/Documents/github-repos/clara/build-debug /home/rewrite/Documents/github-repos/clara/build-debug/library /home/rewrite/Documents/github-repos/clara/build-debug/library/CMakeFiles/clara.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : library/CMakeFiles/clara.dir/depend

