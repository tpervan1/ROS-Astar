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
CMAKE_SOURCE_DIR = /home/tarik/Desktop/MR/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tarik/Desktop/MR/catkin_ws/build

# Include any dependencies generated for this target.
include seminarski_mr/CMakeFiles/listener.dir/depend.make

# Include the progress variables for this target.
include seminarski_mr/CMakeFiles/listener.dir/progress.make

# Include the compile flags for this target's objects.
include seminarski_mr/CMakeFiles/listener.dir/flags.make

seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.o: seminarski_mr/CMakeFiles/listener.dir/flags.make
seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.o: /home/tarik/Desktop/MR/catkin_ws/src/seminarski_mr/src/listener.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tarik/Desktop/MR/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.o"
	cd /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/listener.dir/src/listener.cpp.o -c /home/tarik/Desktop/MR/catkin_ws/src/seminarski_mr/src/listener.cpp

seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/listener.dir/src/listener.cpp.i"
	cd /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tarik/Desktop/MR/catkin_ws/src/seminarski_mr/src/listener.cpp > CMakeFiles/listener.dir/src/listener.cpp.i

seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/listener.dir/src/listener.cpp.s"
	cd /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tarik/Desktop/MR/catkin_ws/src/seminarski_mr/src/listener.cpp -o CMakeFiles/listener.dir/src/listener.cpp.s

# Object files for target listener
listener_OBJECTS = \
"CMakeFiles/listener.dir/src/listener.cpp.o"

# External object files for target listener
listener_EXTERNAL_OBJECTS =

/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: seminarski_mr/CMakeFiles/listener.dir/src/listener.cpp.o
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: seminarski_mr/CMakeFiles/listener.dir/build.make
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/libroscpp.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/librosconsole.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/librostime.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /opt/ros/noetic/lib/libcpp_common.so
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_gapi.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_stitching.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_alphamat.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_aruco.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_bgsegm.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_bioinspired.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_ccalib.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_cvv.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_dnn_objdetect.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_dnn_superres.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_dpm.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_face.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_freetype.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_fuzzy.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_hdf.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_hfs.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_img_hash.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_intensity_transform.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_line_descriptor.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_mcc.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_quality.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_rapid.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_reg.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_rgbd.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_saliency.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_sfm.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_signal.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_stereo.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_structured_light.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_superres.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_surface_matching.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_tracking.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_videostab.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_viz.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_wechat_qrcode.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_xfeatures2d.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_xobjdetect.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_xphoto.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_shape.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_highgui.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_datasets.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_plot.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_text.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_ml.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_phase_unwrapping.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_optflow.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_ximgproc.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_video.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_videoio.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_objdetect.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_calib3d.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_dnn.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_features2d.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_flann.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_photo.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_imgproc.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: /usr/local/lib/libopencv_core.so.4.10.0
/home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener: seminarski_mr/CMakeFiles/listener.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tarik/Desktop/MR/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener"
	cd /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/listener.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
seminarski_mr/CMakeFiles/listener.dir/build: /home/tarik/Desktop/MR/catkin_ws/devel/lib/seminarski_mr/listener

.PHONY : seminarski_mr/CMakeFiles/listener.dir/build

seminarski_mr/CMakeFiles/listener.dir/clean:
	cd /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr && $(CMAKE_COMMAND) -P CMakeFiles/listener.dir/cmake_clean.cmake
.PHONY : seminarski_mr/CMakeFiles/listener.dir/clean

seminarski_mr/CMakeFiles/listener.dir/depend:
	cd /home/tarik/Desktop/MR/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tarik/Desktop/MR/catkin_ws/src /home/tarik/Desktop/MR/catkin_ws/src/seminarski_mr /home/tarik/Desktop/MR/catkin_ws/build /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr /home/tarik/Desktop/MR/catkin_ws/build/seminarski_mr/CMakeFiles/listener.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : seminarski_mr/CMakeFiles/listener.dir/depend

