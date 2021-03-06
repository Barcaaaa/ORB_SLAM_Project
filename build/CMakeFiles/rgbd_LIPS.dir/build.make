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
CMAKE_SOURCE_DIR = /home/wuyao818/SLAM/Project/ORB-SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wuyao818/SLAM/Project/ORB-SLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/rgbd_LIPS.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rgbd_LIPS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rgbd_LIPS.dir/flags.make

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o: CMakeFiles/rgbd_LIPS.dir/flags.make
CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o: ../Examples/RGB-D/rgbd_LIPS.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wuyao818/SLAM/Project/ORB-SLAM/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o -c /home/wuyao818/SLAM/Project/ORB-SLAM/Examples/RGB-D/rgbd_LIPS.cc

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wuyao818/SLAM/Project/ORB-SLAM/Examples/RGB-D/rgbd_LIPS.cc > CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.i

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wuyao818/SLAM/Project/ORB-SLAM/Examples/RGB-D/rgbd_LIPS.cc -o CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.s

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.requires:
.PHONY : CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.requires

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.provides: CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.requires
	$(MAKE) -f CMakeFiles/rgbd_LIPS.dir/build.make CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.provides.build
.PHONY : CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.provides

CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.provides.build: CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o

# Object files for target rgbd_LIPS
rgbd_LIPS_OBJECTS = \
"CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o"

# External object files for target rgbd_LIPS
rgbd_LIPS_EXTERNAL_OBJECTS =

../Examples/RGB-D/rgbd_LIPS: CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o
../Examples/RGB-D/rgbd_LIPS: CMakeFiles/rgbd_LIPS.dir/build.make
../Examples/RGB-D/rgbd_LIPS: ../lib/libORB_SLAM2.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_viz.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_videostab.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_ts.a
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_superres.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_stitching.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_contrib.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_nonfree.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_ocl.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_gpu.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_photo.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_objdetect.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_legacy.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_video.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_ml.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_calib3d.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_features2d.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_highgui.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_imgproc.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_flann.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libopencv_core.so.2.4.13
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpangolin.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/RGB-D/rgbd_LIPS: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/RGB-D/rgbd_LIPS: ../Thirdparty/g2o/lib/libg2o.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_surface.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_recognition.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_registration.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_keypoints.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_tracking.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_stereo.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_outofcore.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_people.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_visualization.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_io.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_segmentation.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_features.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_filters.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_sample_consensus.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_search.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_octree.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_kdtree.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_ml.so
../Examples/RGB-D/rgbd_LIPS: /usr/local/lib/libpcl_common.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_system.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libpthread.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libqhull.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libOpenNI.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libOpenNI2.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkGenericFiltering.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkGeovis.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkCharts.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkViews.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkInfovis.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkWidgets.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkVolumeRendering.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkHybrid.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkParallel.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkRendering.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkImaging.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkGraphics.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkIO.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkFiltering.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtkCommon.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: /usr/lib/libvtksys.so.5.8.0
../Examples/RGB-D/rgbd_LIPS: CMakeFiles/rgbd_LIPS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../Examples/RGB-D/rgbd_LIPS"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd_LIPS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rgbd_LIPS.dir/build: ../Examples/RGB-D/rgbd_LIPS
.PHONY : CMakeFiles/rgbd_LIPS.dir/build

CMakeFiles/rgbd_LIPS.dir/requires: CMakeFiles/rgbd_LIPS.dir/Examples/RGB-D/rgbd_LIPS.cc.o.requires
.PHONY : CMakeFiles/rgbd_LIPS.dir/requires

CMakeFiles/rgbd_LIPS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rgbd_LIPS.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rgbd_LIPS.dir/clean

CMakeFiles/rgbd_LIPS.dir/depend:
	cd /home/wuyao818/SLAM/Project/ORB-SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wuyao818/SLAM/Project/ORB-SLAM /home/wuyao818/SLAM/Project/ORB-SLAM /home/wuyao818/SLAM/Project/ORB-SLAM/build /home/wuyao818/SLAM/Project/ORB-SLAM/build /home/wuyao818/SLAM/Project/ORB-SLAM/build/CMakeFiles/rgbd_LIPS.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rgbd_LIPS.dir/depend

