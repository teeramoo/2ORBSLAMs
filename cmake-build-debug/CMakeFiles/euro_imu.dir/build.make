# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/teeramoo/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/182.3911.40/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/teeramoo/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/182.3911.40/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/euro_imu.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/euro_imu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/euro_imu.dir/flags.make

CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o: CMakeFiles/euro_imu.dir/flags.make
CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o: ../Examples/Monocular/euro_imu.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o -c /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/Examples/Monocular/euro_imu.cc

CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/Examples/Monocular/euro_imu.cc > CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.i

CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/Examples/Monocular/euro_imu.cc -o CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.s

# Object files for target euro_imu
euro_imu_OBJECTS = \
"CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o"

# External object files for target euro_imu
euro_imu_EXTERNAL_OBJECTS =

../Examples/Monocular/euro_imu: CMakeFiles/euro_imu.dir/Examples/Monocular/euro_imu.cc.o
../Examples/Monocular/euro_imu: CMakeFiles/euro_imu.dir/build.make
../Examples/Monocular/euro_imu: ../lib/libORB_SLAM2.so
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_stitching.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_superres.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_videostab.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_photo.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_aruco.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_bgsegm.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_bioinspired.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_ccalib.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_dpm.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_face.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_freetype.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_fuzzy.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_line_descriptor.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_optflow.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_reg.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_rgbd.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_saliency.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_sfm.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_stereo.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_structured_light.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_viz.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_phase_unwrapping.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_surface_matching.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_tracking.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_datasets.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_dnn.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_plot.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_text.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_ml.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_xfeatures2d.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_shape.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_video.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_ximgproc.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_calib3d.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_features2d.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_flann.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_highgui.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_videoio.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_xobjdetect.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_objdetect.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_xphoto.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_imgproc.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libopencv_core.so.3.2.0
../Examples/Monocular/euro_imu: /usr/local/lib/libpangolin.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libpython3.5m.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libavformat.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libavutil.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libswscale.so
../Examples/Monocular/euro_imu: /usr/lib/libOpenNI.so
../Examples/Monocular/euro_imu: /usr/lib/libOpenNI2.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/Monocular/euro_imu: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Monocular/euro_imu: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libcholmod.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libamd.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libcolamd.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libcamd.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libccolamd.so
../Examples/Monocular/euro_imu: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../Examples/Monocular/euro_imu: CMakeFiles/euro_imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Monocular/euro_imu"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/euro_imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/euro_imu.dir/build: ../Examples/Monocular/euro_imu

.PHONY : CMakeFiles/euro_imu.dir/build

CMakeFiles/euro_imu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/euro_imu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/euro_imu.dir/clean

CMakeFiles/euro_imu.dir/depend:
	cd /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug /home/teeramoo/Desktop/Thesis_work/LearnVIORB-master/cmake-build-debug/CMakeFiles/euro_imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/euro_imu.dir/depend

