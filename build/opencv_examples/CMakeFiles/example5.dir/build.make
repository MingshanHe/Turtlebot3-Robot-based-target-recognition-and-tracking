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
CMAKE_SOURCE_DIR = /home/hemingshan/opencv_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hemingshan/opencv_ws/build

# Include any dependencies generated for this target.
include opencv_examples/CMakeFiles/example5.dir/depend.make

# Include the progress variables for this target.
include opencv_examples/CMakeFiles/example5.dir/progress.make

# Include the compile flags for this target's objects.
include opencv_examples/CMakeFiles/example5.dir/flags.make

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o: opencv_examples/CMakeFiles/example5.dir/flags.make
opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o: /home/hemingshan/opencv_ws/src/opencv_examples/src/example5.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hemingshan/opencv_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o"
	cd /home/hemingshan/opencv_ws/build/opencv_examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example5.dir/src/example5.cc.o -c /home/hemingshan/opencv_ws/src/opencv_examples/src/example5.cc

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example5.dir/src/example5.cc.i"
	cd /home/hemingshan/opencv_ws/build/opencv_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hemingshan/opencv_ws/src/opencv_examples/src/example5.cc > CMakeFiles/example5.dir/src/example5.cc.i

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example5.dir/src/example5.cc.s"
	cd /home/hemingshan/opencv_ws/build/opencv_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hemingshan/opencv_ws/src/opencv_examples/src/example5.cc -o CMakeFiles/example5.dir/src/example5.cc.s

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.requires:

.PHONY : opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.requires

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.provides: opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.requires
	$(MAKE) -f opencv_examples/CMakeFiles/example5.dir/build.make opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.provides.build
.PHONY : opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.provides

opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.provides.build: opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o


# Object files for target example5
example5_OBJECTS = \
"CMakeFiles/example5.dir/src/example5.cc.o"

# External object files for target example5
example5_EXTERNAL_OBJECTS =

/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: opencv_examples/CMakeFiles/example5.dir/build.make
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libcv_bridge.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libimage_transport.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libmessage_filters.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libclass_loader.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/libPocoFoundation.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libdl.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libroslib.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/librospack.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libroscpp.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/librosconsole.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/librostime.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /opt/ros/melodic/lib/libcpp_common.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5: opencv_examples/CMakeFiles/example5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hemingshan/opencv_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5"
	cd /home/hemingshan/opencv_ws/build/opencv_examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
opencv_examples/CMakeFiles/example5.dir/build: /home/hemingshan/opencv_ws/devel/lib/opencv_examples/example5

.PHONY : opencv_examples/CMakeFiles/example5.dir/build

opencv_examples/CMakeFiles/example5.dir/requires: opencv_examples/CMakeFiles/example5.dir/src/example5.cc.o.requires

.PHONY : opencv_examples/CMakeFiles/example5.dir/requires

opencv_examples/CMakeFiles/example5.dir/clean:
	cd /home/hemingshan/opencv_ws/build/opencv_examples && $(CMAKE_COMMAND) -P CMakeFiles/example5.dir/cmake_clean.cmake
.PHONY : opencv_examples/CMakeFiles/example5.dir/clean

opencv_examples/CMakeFiles/example5.dir/depend:
	cd /home/hemingshan/opencv_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hemingshan/opencv_ws/src /home/hemingshan/opencv_ws/src/opencv_examples /home/hemingshan/opencv_ws/build /home/hemingshan/opencv_ws/build/opencv_examples /home/hemingshan/opencv_ws/build/opencv_examples/CMakeFiles/example5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencv_examples/CMakeFiles/example5.dir/depend

