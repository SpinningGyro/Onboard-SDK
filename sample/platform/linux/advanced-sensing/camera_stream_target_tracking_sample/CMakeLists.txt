cmake_minimum_required(VERSION 2.8)
project(camera-stream-target-tracking-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

# FFmpeg-streamer
add_subdirectory(streamer)
set(FFMPEG_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
set(FFMPEG_LIBRARIES /usr/lib/aarch64-linux-gnu)  
set(FFMPEG_avformat_LIBRARY /usr/lib/aarch64-linux-gnu/libavformat.so)
set(FFMPEG_avcodec_LIBRARY /usr/lib/aarch64-linux-gnu/libavcodec.so)
set(FFMPEG_avutil_LIBRARY /usr/lib/aarch64-linux-gnu/libavutil.so)
set(SWSCALE_LIBRARY /usr/lib/aarch64-linux-gnu/libswscale.so)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(AVUTIL_LIBRARY avutil)
find_library(SWSCALE_LIBRARY swscale)
MESSAGE(${AVFORMAT_LIBRARY})

# OpenCV
# Try to see if user has OpenCV installed
ADD_DEFINITIONS(-DOPENCV)
#set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv-4.1.0/build/)
set(OpenCV_DIR "/home/nvidia/opencv-4.1.0/build/")
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message( "\n${PROJECT_NAME}...")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
else()
  message( STATUS "OpenCV is required for this sample to do the target tracking")
endif ()

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message( "\n${PROJECT_NAME}...")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
else()
  message( STATUS "OpenCV is required for this sample to do the target tracking")
endif ()


# darknet
if(NOT DARKNET_LIBRARY)
set(DARKNET_LIBRARY /home/nvidia/darknet/libdarknet.so)
endif()

unset(KCFcpp_DIR)
find_path(KCFcpp_DIR
        NAMES
        KCFCpp.sh
        PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/KCFcpp
        )

if(NOT KCFcpp_DIR)
  message(STATUS "Target Tracking sample depends on KCFcpp library from https://github.com/joaofaro/KCFcpp.git")
  message(STATUS "When you run make, we will try to clone the repo")
  message(STATUS "This will happen only once as long as you do not delete it")

  include(${CMAKE_MODULE_PATH}/DownloadProject/DownloadProject.cmake)
  download_project(PROJ                kcf_cpp
          GIT_REPOSITORY      https://github.com/joaofaro/KCFcpp.git
          GIT_TAG             master
          ${UPDATE_DISCONNECTED_IF_AVAILABLE}
          SOURCE_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/KCFcpp
          )
endif()

set(ONBOARDSDK_SOURCE /home/nvidia/DJI/Onboard-SDK/osdk-core)
include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker/armv8/inc)
include_directories(${ONBOARDSDK_SOURCE}/logger/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${ONBOARDSDK_SOURCE}/modules/inc/payload)
include_directories(${ONBOARDSDK_SOURCE}/modules/inc/flight)
include_directories(${ONBOARDSDK_SOURCE}/modules/inc/filemgr)
include_directories(${ONBOARDSDK_SOURCE}/modules/inc/mop)
include_directories(${ONBOARDSDK_SOURCE}/modules/inc/firewall)
include_directories(${ONBOARDSDK_SOURCE}/../sample/platform/linux/common)
include_directories(${ONBOARDSDK_SOURCE}/../sample/platform/linux/hal)
include_directories(${ONBOARDSDK_SOURCE}/../sample/platform/linux/osal)
include_directories(${ONBOARDSDK_SOURCE}/../sample/core/inc)

## To use this this repo, you need to run
##  git clone https://github.com/joaofaro/KCFcpp.git
## under the current foler
include_directories(KCFcpp/src)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(kcf_tracker STATIC
  KCFcpp/src/fhog.cpp
  KCFcpp/src/kcftracker.cpp
  YoloObjectDetector.cpp)
target_link_libraries(kcf_tracker ${OpenCV_LIBRARIES} streamer ${OpenCV_LIBS} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${SWSCALE_LIBRARY} ${DARKNET_LIBRARY})

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
add_executable(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
  ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp
  ${ONBOARDSDK_SOURCE}/../sample/core/src/gimbal_manager_sync_sample.cpp
  ${ONBOARDSDK_SOURCE}/../sample/core/src/camera_manager_sync_sample.cpp
  # target_tracking.cpp
  planning.cpp
  communicator.cpp
  tracking_utility.cpp
  main-merge.cpp
  )

target_link_libraries(${PROJECT_NAME}
  kcf_tracker
  ${OpenCV_LIBRARIES}
  crypto
  )
