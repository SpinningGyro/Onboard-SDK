/*
 * @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "dji_vehicle.hpp"
#include <iostream>
#include "dji_linux_helpers.hpp"
#include "tracking_utility.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kcftracker.hpp"

#include "camera_manager_sync_sample.hpp"
#include "gimbal_manager_sync_sample.hpp"

#include "streamer/streamer.hpp"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <math.h>
#include <cstdio>
#include <chrono>
#include <exception> 

#include "planning.hpp"
#include "YoloObjectDetector.hpp"

#define DEG2RAD 0.01745329252

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;
using namespace cv;
using namespace streamer;
using namespace detector;

void process_frame(const cv::Mat &in, cv::Mat &out)
{
    in.copyTo(out);
}

void stream_frame(Streamer &streamer, const cv::Mat &image)
{
    streamer.stream_frame(image.data);
}

// 初始化推流器
int startStreamer(Vehicle* vehicle, Streamer &streamer){

  int cap_frame_width = 0,cap_frame_height=0,stream_fps=24,bitrate=20000000;
  int ret = 0;
  int retryTimes = 20; // 重试次数
  int count = 0;
  CameraRGBImage mainImg;
  while(count < retryTimes)
  {
    if(vehicle->advancedSensing->newMainCameraImageReady())
    {      
      vehicle->advancedSensing->getMainCameraImage(mainImg);
      cap_frame_width = mainImg.width;
      cap_frame_height = mainImg.height;
      cout << "Open RTMP..." << endl;
      cout << "Height: " << cap_frame_height << " Width: " << cap_frame_width << endl;
      try
      {
        StreamerConfig streamer_config(cap_frame_width, cap_frame_height,
                                   cap_frame_width, cap_frame_height,
                                   stream_fps, bitrate, "main", "rtmp://221.218.247.38:1935/live/m300");
                                  //  stream_fps, bitrate, "main", "rtmp://192.168.1.31:1935/live/m210");
        cout << "init streamer..." << endl;
        streamer.init(streamer_config);
        cout << "init streamer finished." << endl;
        ret = 1;
        break;
        
      }
      catch(char *str)
      {
        cout << "Exception caught: " << str << endl;
      }
      count++;      
    }
  }
  return ret;
}

bool setUpSubscription(Vehicle* vehicle,int timeout) {
  const int DEFAULT_PACKAGE_INDEX = 0;
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehicle->subscribe->verify(timeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  //TOPIC_ALTITUDE_OF_HOMEPOINT, TOPIC_GPS_POSITION
  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int freq = 1;
  TopicName topicList10Hz[] = {TOPIC_GPS_FUSED };
  int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = true;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus)) {
    return pkgStatus;
  }
  usleep(5000);
  // Start listening to the telemetry data
  subscribeStatus =
    vehicle->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, timeout);
  usleep(5000);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(
        DEFAULT_PACKAGE_INDEX, timeout);
    if (ACK::getError(ack)) {
      DSTATUS(
          "Error unsubscribing; please restart the drone/FC to get "
          "back to a clean state.\n");
    }
    return false;
  }
  return true;
}

int checkSetup(Vehicle*   vehicle){
  if (vehicle == NULL)
  {
    DERROR("Vehicle not initialized, exiting.\n");
    return  -1;
  }

  vehicle->advancedSensing->stopMainCameraStream();

  bool mainCamResult = vehicle->advancedSensing->startMainCameraStream();
  if(!mainCamResult)
  {
    DERROR("Failed to Open Camera\n");
    return -2;
  }

  if(!vehicle->gimbal)
  {
    DERROR("Gimbal object does not exist.\n");
    return -3;
  }
  int timeout = 1;
  if(!setUpSubscription(vehicle,timeout)){
    return -4;
  }
  return 1;
}

int main(int argc, char** argv)
{
  //Setup detector
  string  names_file = "/home/nvidia/darknet/data/voc.names";
  string  cfg_file = "/home/nvidia/darknet/cfg/yolov3-tiny-prn.cfg";
  string  weights_file = "/home/nvidia/darknet/backup/yolov3-tiny-prn_best.weights";
  // string video_file = "/home/breeze/MyCode/darknet_streaming/build/DJI_0318.mp4";
  //初始化检测器
  Mat result_frame;
  YoloObjectDetector* yolo_detector = new YoloObjectDetector(names_file,cfg_file,weights_file);

  // Setup OSDK.
  bool enableAdvancedSensing = true;
  LinuxSetup linuxEnvironment(argc, argv, enableAdvancedSensing);
  Vehicle*   vehicle;
  int retryTimes = 20;
  for(int i=0;i < retryTimes;i++){
    DSTATUS("[%d/%d] 启动 OSDK 环境",i,retryTimes);
    
    vehicle = linuxEnvironment.getVehicle();
    const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
    vehicle->advancedSensing->setAcmDevicePath(acm_dev);

    int ret = checkSetup(vehicle);
    if(ret) break;
  }
  DSTATUS("启动 OSDK 环境 成功");
  
  // 获取控制权时无法控制云台（3.9),(4.0未测试)
  // // Initialize variables
  // int functionTimeout = 1;
  // // Obtain Control Authority
  // vehicle->obtainCtrlAuthority(functionTimeout);

  // // 启动推流器
  // bool isPushStream = 0;
  // cv::Mat proc_frame;
  // Streamer streamer;
  // int ret = startStreamer(vehicle,streamer);
  // if(ret){
  //   isPushStream = 1;
  // }

  /*! Create an example object, which users can modify according to their own needs */
  // 相机管理
  CameraManagerSyncSample *cam = new CameraManagerSyncSample(vehicle);
  // 云台管理
  GimbalManagerSyncSample *gim = new GimbalManagerSyncSample(vehicle);

  CameraRGBImage mainImg;
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);

  cv::namedWindow(winName,1);
  

  float64_t currentLatitude;
  float64_t currentLongitude;
  float64_t currentAltitude;
  std::string wpFile = "/home/nvidia/mop2.txt";
  std::vector<planningWayPoint> pWps = getPlaningPoint(wpFile);
  std::cout << "航点数： " << pWps.size() << endl;

  while(true)
  {
    // Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    // currentLatitude  = subscribeGPosition.latitude;
    // currentLongitude = subscribeGPosition.longitude;

    // Start listening to the telemetry data

    // ACK::ErrorCode subscribeStatus = vehicle->subscribe->startPackage(0, 1);

    // Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    // currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    // currentLatitude = currentSubscriptionGPS.latitude;
    // currentLongitude = currentSubscriptionGPS.longitude;
    // currentAltitude = currentSubscriptionGPS.altitude - 50;

    Telemetry::GlobalPosition globalPosition = vehicle->broadcast->getGlobalPosition();
    currentLatitude = globalPosition.latitude / DEG2RAD;
    currentLongitude = globalPosition.longitude / DEG2RAD;
    currentAltitude = globalPosition.altitude - 50;

    std::cout << "currentLatitude: " << globalPosition.latitude << 
              "  currentLongitude: " << globalPosition.longitude << 
              " currentAltitude" << currentAltitude << std::endl;

    std::cout << "currentLatitude: " << currentLatitude << "  currentLongitude: " << currentLongitude << "  currentAltitude: " << currentAltitude << std::endl;

    float32_t distanceLimit = 10;
    bool isDetect = false;
    if(isInPlanningRang(currentLongitude,currentLatitude,currentAltitude,pWps,10)){
      isDetect = true;
    }
    if(vehicle->advancedSensing->newMainCameraImageReady() && isDetect)
    {
      int count = 0;
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      // 获取云台初始角度

      cout << "获取云台初始角度" << endl;
      GimbalSingleData initialGimbalData = gim->getGimbalData(PAYLOAD_INDEX_0);

      DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
                initialGimbalData.pitch,
                initialGimbalData.roll,
                initialGimbalData.yaw); 

      // // 先中心对焦
      // cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);   

      // 累积多帧图片的检测结果
      vector<Rect> collect_rois, keep_rois;
      int accumulated_frame_count = 20;
      int filter_thresh = (int)(accumulated_frame_count * 0.6);
      for(int i=0;i<accumulated_frame_count;i++){
        vehicle->advancedSensing->getMainCameraImage(mainImg);
        Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
        //进行目标检测
        vector<Rect> rois = yolo_detector->detect_bboxes(frame,frame,0.2);
        cvtColor(frame, frame, COLOR_RGB2BGR);
        cv::imshow(winName, frame);
        cv::waitKey(1);
        collect_rois.insert(collect_rois.end(),rois.begin(),rois.end());
      }
      std::cout << "累积目标数：" << collect_rois.size() << std::endl;
      keep_rois = do_merge_nms(collect_rois,0.1,filter_thresh);
      std::cout << "检测到目标数：" << keep_rois.size() << std::endl;

      // for(auto &roi:keep_rois){
       
      //   std::cout << "拍照目标..." << std::endl;
      //   std::cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;
      // }

      for(auto &roi:keep_rois){        
        vehicle->advancedSensing->getMainCameraImage(mainImg);
        Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);    
        
        std::cout << "执行拍照..." << std::endl;
        std::cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;

        for(auto &kroi:keep_rois){
            cv::Rect kroi2(kroi.x - dx,kroi.x + dy,kroi.width,kroi.height);
            cv::rectangle(frame, kroi, cv::Scalar(0,0,255), 1, 8, 0 );
        }

        cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
        if(roi.width != 0)
        {
          cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

          cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
                  Point(roi.x + roi.width/2, roi.y + roi.height/2),
                  cv::Scalar(0,255,255));
        }
        cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
        cvtColor(frame, frame, COLOR_RGB2BGR);
        
        // send gimbal speed command
        dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
        dy = (int)(roi.y + roi.height/2 - mainImg.height/2);
        sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
        putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        cv::imshow(winName, frame);
        cv::waitKey(1);  

        // send gimbal speed command
        dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
        dy = (int)(roi.y + roi.height/2 - mainImg.height/2);

        yawRate   = dx;
        pitchRate = -dy;

        if(abs(dx) < 10)
        {
          yawRate = 0;
        }

        if(abs(dy) < 10)
        {
          pitchRate = 0;
        }

        DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
        gimbalSpeed.roll     = 0;
        gimbalSpeed.pitch    = pitchRate;
        gimbalSpeed.yaw      = yawRate;
        gimbalSpeed.gimbal_control_authority = 1;

        vehicle->gimbal->setSpeed(&gimbalSpeed);

        usleep(1000000);

        float w_scale = (mainImg.width-50) / roi.width;
        float h_scale = (mainImg.height-50) / roi.height;
        float scale = w_scale < h_scale ? w_scale:h_scale;

        std::cout << "图像大小：" << mainImg.width << " " << mainImg.height << endl;
        std::cout << "变焦倍数：" << scale << std::endl;

        // cam->setZoomSyncSample(PAYLOAD_INDEX_0, 10);
        cam->setZoomSyncSample(PAYLOAD_INDEX_0, scale);
        sleep(1);
        cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);
        usleep(1000000);
        cam->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);
        usleep(1000000);
        cam->setZoomSyncSample(PAYLOAD_INDEX_0, 2);
        
        GimbalModule::Rotation rotation;
        rotation.roll = 0.0f;
        rotation.pitch = initialGimbalData.pitch;
        rotation.yaw = initialGimbalData.yaw;
        rotation.rotationMode = 0;
        rotation.time = 0.5;
        gim->rotateSyncSample(PAYLOAD_INDEX_0, rotation);
        sleep(1);

      }
      cv::waitKey(1);  

    }
  }

  vehicle->advancedSensing->stopMainCameraStream();

  return 0;
}
