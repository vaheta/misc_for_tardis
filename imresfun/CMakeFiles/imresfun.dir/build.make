# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/master/Documents/mylibs/imresfun

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/master/Documents/mylibs/imresfun

# Include any dependencies generated for this target.
include CMakeFiles/imresfun.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imresfun.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imresfun.dir/flags.make

CMakeFiles/imresfun.dir/imresfun.cpp.o: CMakeFiles/imresfun.dir/flags.make
CMakeFiles/imresfun.dir/imresfun.cpp.o: imresfun.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/master/Documents/mylibs/imresfun/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/imresfun.dir/imresfun.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/imresfun.dir/imresfun.cpp.o -c /home/master/Documents/mylibs/imresfun/imresfun.cpp

CMakeFiles/imresfun.dir/imresfun.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imresfun.dir/imresfun.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/master/Documents/mylibs/imresfun/imresfun.cpp > CMakeFiles/imresfun.dir/imresfun.cpp.i

CMakeFiles/imresfun.dir/imresfun.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imresfun.dir/imresfun.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/master/Documents/mylibs/imresfun/imresfun.cpp -o CMakeFiles/imresfun.dir/imresfun.cpp.s

CMakeFiles/imresfun.dir/imresfun.cpp.o.requires:
.PHONY : CMakeFiles/imresfun.dir/imresfun.cpp.o.requires

CMakeFiles/imresfun.dir/imresfun.cpp.o.provides: CMakeFiles/imresfun.dir/imresfun.cpp.o.requires
	$(MAKE) -f CMakeFiles/imresfun.dir/build.make CMakeFiles/imresfun.dir/imresfun.cpp.o.provides.build
.PHONY : CMakeFiles/imresfun.dir/imresfun.cpp.o.provides

CMakeFiles/imresfun.dir/imresfun.cpp.o.provides.build: CMakeFiles/imresfun.dir/imresfun.cpp.o

# Object files for target imresfun
imresfun_OBJECTS = \
"CMakeFiles/imresfun.dir/imresfun.cpp.o"

# External object files for target imresfun
imresfun_EXTERNAL_OBJECTS =

imresfun: CMakeFiles/imresfun.dir/imresfun.cpp.o
imresfun: CMakeFiles/imresfun.dir/build.make
imresfun: /usr/local/lib/libopencv_videostab.so.3.0.0
imresfun: /usr/local/lib/libopencv_videoio.so.3.0.0
imresfun: /usr/local/lib/libopencv_video.so.3.0.0
imresfun: /usr/local/lib/libopencv_superres.so.3.0.0
imresfun: /usr/local/lib/libopencv_stitching.so.3.0.0
imresfun: /usr/local/lib/libopencv_shape.so.3.0.0
imresfun: /usr/local/lib/libopencv_photo.so.3.0.0
imresfun: /usr/local/lib/libopencv_objdetect.so.3.0.0
imresfun: /usr/local/lib/libopencv_ml.so.3.0.0
imresfun: /usr/local/lib/libopencv_imgproc.so.3.0.0
imresfun: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
imresfun: /usr/local/lib/libopencv_highgui.so.3.0.0
imresfun: /usr/local/lib/libopencv_hal.a
imresfun: /usr/local/lib/libopencv_flann.so.3.0.0
imresfun: /usr/local/lib/libopencv_features2d.so.3.0.0
imresfun: /usr/local/lib/libopencv_core.so.3.0.0
imresfun: /usr/local/lib/libopencv_calib3d.so.3.0.0
imresfun: /usr/local/lib/libopencv_features2d.so.3.0.0
imresfun: /usr/local/lib/libopencv_ml.so.3.0.0
imresfun: /usr/local/lib/libopencv_highgui.so.3.0.0
imresfun: /usr/local/lib/libopencv_videoio.so.3.0.0
imresfun: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
imresfun: /usr/local/lib/libopencv_flann.so.3.0.0
imresfun: /usr/local/lib/libopencv_video.so.3.0.0
imresfun: /usr/local/lib/libopencv_imgproc.so.3.0.0
imresfun: /usr/local/lib/libopencv_core.so.3.0.0
imresfun: /usr/local/lib/libopencv_hal.a
imresfun: /usr/local/share/OpenCV/3rdparty/lib/libippicv.a
imresfun: CMakeFiles/imresfun.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable imresfun"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imresfun.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imresfun.dir/build: imresfun
.PHONY : CMakeFiles/imresfun.dir/build

CMakeFiles/imresfun.dir/requires: CMakeFiles/imresfun.dir/imresfun.cpp.o.requires
.PHONY : CMakeFiles/imresfun.dir/requires

CMakeFiles/imresfun.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imresfun.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imresfun.dir/clean

CMakeFiles/imresfun.dir/depend:
	cd /home/master/Documents/mylibs/imresfun && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/master/Documents/mylibs/imresfun /home/master/Documents/mylibs/imresfun /home/master/Documents/mylibs/imresfun /home/master/Documents/mylibs/imresfun /home/master/Documents/mylibs/imresfun/CMakeFiles/imresfun.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imresfun.dir/depend

