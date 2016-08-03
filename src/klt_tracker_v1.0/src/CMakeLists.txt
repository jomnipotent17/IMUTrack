cmake_minimum_required(VERSION 2.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


cuda_add_library(klt_tracker_lib featurePool.cxx  featureTrack.cxx  imageAlign.cxx  cameraModel.cxx cuda_feature_select.cxx  cuda_image_align.cu)
add_executable(klt_tracker main_tracker.cxx  display_tracker.cxx  playback_tracker.cxx)
target_link_libraries(klt_tracker klt_tracker_lib ${OpenCV_LIBS} ${IPP_LIBS})

#target_link_libraries(OpenCV affineTracker_gpu)
