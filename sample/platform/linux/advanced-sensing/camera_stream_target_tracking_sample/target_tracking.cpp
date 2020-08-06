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

#include "streamer/streamer.hpp"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <cstdio>
#include <chrono>
#include <exception> 

#include "YoloObjectDetector.hpp"

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

// Be precise here
struct RotationAngle
{
  DJI::OSDK::float32_t roll;
  DJI::OSDK::float32_t pitch;
  DJI::OSDK::float32_t yaw;
};

struct GimbalContainer
{
  int           roll             = 0;
  int           pitch            = 0;
  int           yaw              = 0;
  int           duration         = 0;
  int           isAbsolute       = 0;
  bool          yaw_cmd_ignore   = false;
  bool          pitch_cmd_ignore = false;
  bool          roll_cmd_ignore  = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0,
                  int isAbsolute = 0, RotationAngle initialAngle = {},
                  RotationAngle currentAngle = {})
    : roll(roll)
    , pitch(pitch)
    , yaw(yaw)
    , duration(duration)
    , isAbsolute(isAbsolute)
    , initialAngle(initialAngle)
    , currentAngle(currentAngle)
  {
  }
};

void doSetGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal)
{
  DJI::OSDK::Gimbal::AngleData gimbalAngle = {};
  gimbalAngle.roll     = gimbal->roll;
  gimbalAngle.pitch    = gimbal->pitch;
  gimbalAngle.yaw      = gimbal->yaw;
  gimbalAngle.duration = gimbal->duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal->isAbsolute;
  gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

  vehicle->gimbal->setAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(2);
}

void displayResult(RotationAngle* currentAngle)
{
  std::cout << "New Gimbal rotation angle is [";
  std::cout << currentAngle->roll << " ";
  std::cout << currentAngle->pitch << " ";
  std::cout << currentAngle->yaw;
  std::cout << "]\n\n";
}

int main(int argc, char** argv)
{
  //Setup detector
  string  names_file = "/home/nvidia/darknet/data/voc.names";
  string  cfg_file = "/home/nvidia/darknet/cfg/yolov3-tiny-prn.cfg";
  string  weights_file = "/home/nvidia/darknet/backup/yolov3-tiny-prn_best.weights";
  string video_file = "/home/breeze/MyCode/darknet_streaming/build/DJI_0318.mp4";
  //初始化检测器
  Mat result_frame;
  YoloObjectDetector yolo_detector(names_file,cfg_file,weights_file);

  // Setup OSDK.
  bool enableAdvancedSensing = true;
  LinuxSetup linuxEnvironment(argc, argv, enableAdvancedSensing);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
  vehicle->advancedSensing->setAcmDevicePath(acm_dev);
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // // Initialize variables
  // int functionTimeout = 1;
  // // Obtain Control Authority
  // vehicle->obtainCtrlAuthority(functionTimeout);

  bool mainCamResult = vehicle->advancedSensing->startMainCameraStream();
  if(!mainCamResult)
  {
    cout << "Failed to Open Camera" << endl;
    return 1;
  }

  CameraRGBImage mainImg;
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);

  KCFTracker *tracker = NULL;
  TrackingUtility tu;

  cv::namedWindow(winName,1);
  cv::setMouseCallback(winName,TrackingUtility::mouseCallback, (void*)&tu);
  
  //Setp rtmp 
  cv::Mat proc_frame;
  Streamer streamer;
  int cap_frame_width = 0,cap_frame_height=0,stream_fps=24,bitrate=20000000;
  // while(1)
  // {
  //   if(vehicle->advancedSensing->newMainCameraImageReady())
  //   {
      
  //     vehicle->advancedSensing->getMainCameraImage(mainImg);
  //     cap_frame_width = mainImg.width;
  //     cap_frame_height = mainImg.height;
  //     cout << "Open RTMP..." << endl;
  //     cout << "Height: " << cap_frame_height << " Width: " << cap_frame_width << endl;
  //     // try
  //     // {
  //     //   StreamerConfig streamer_config(cap_frame_width, cap_frame_height,
  //     //                              cap_frame_width, cap_frame_height,
  //     //                              //stream_fps, bitrate, "main", "rtmp://221.218.247.38:1935/live/m210");
  //     //                              stream_fps, bitrate, "main", "rtmp://192.168.1.31:1935/live/m210");
  //     //   cout << "init streamer..." << endl;
  //     //   streamer.init(streamer_config);
  //     //   cout << "init streamer finished." << endl;
        
  //     // }
  //     // catch(char *str)
  //     // {
  //     //   cout << "Exception caught: " << str << endl;
  //     // }
      
  //     break;
      
  //   }
  // }
  cout << "start tracking..." << endl;
  while(1)
  {
    cout << "tracking..." << endl;
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }

    tu.getKey(c); //Internal states will be updated based on key pressed.

    if(vehicle->advancedSensing->newMainCameraImageReady())
    {
      int count = 0;
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      // 获取云台初始角度
      if(!vehicle->gimbal)
      {
        DERROR("Gimbal object does not exist.\n");
        return false;
      }

      GimbalContainer gimbal;
      RotationAngle initialAngle;
      RotationAngle currentAngle;
      int pkgIndex;
      int responseTimeout = 0;

      /*
      * Subscribe to gimbal data not supported in MAtrice 100
      */
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        cout << "############# subscribe #####################" << endl;
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
          ACK::getErrorCodeMessage(subscribeStatus, __func__);
          return false;
        }

        // Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
        pkgIndex                  = 0;
        int       freq            = 10;
        TopicName topicList10Hz[] = { TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS };
        int       numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool      enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
          pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
          return pkgStatus;
        }
        subscribeStatus =
          vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
          ACK::getErrorCodeMessage(subscribeStatus, __func__);
          // Cleanup before return
          vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
          return false;
        }
      }

      sleep(1);

      cout << "获取云台初始角度" << endl;
      
      // Get Gimbal initial values
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        initialAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
        initialAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
        initialAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
      }
      else
      {
        initialAngle.roll  = vehicle->broadcast->getGimbal().roll;
        initialAngle.pitch = vehicle->broadcast->getGimbal().pitch;
        initialAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
      }

      std::cout << "Initial Gimbal rotation angle: [" << initialAngle.roll << ", "
                << initialAngle.pitch << ", " << initialAngle.yaw << "]\n\n";

      // gimbal = GimbalContainer(initialAngle.roll*10, initialAngle.pitch*10, initialAngle.yaw*10+922, 20, 1, initialAngle);

      // 累积多帧图片的检测结果
      vector<Rect> collect_rois, keep_rois;
      int accumulated_frame_count = 20;
      int filter_thresh = (int)(accumulated_frame_count * 0.6);
      for(int i=0;i<accumulated_frame_count;i++){
        vehicle->advancedSensing->getMainCameraImage(mainImg);
        Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
        //进行目标检测
        vector<Rect> rois = yolo_detector.detect_bboxes(frame,frame,0.2);
        cvtColor(frame, frame, COLOR_RGB2BGR);
        cv::imshow(winName, frame);
        cv::waitKey(1);
        collect_rois.insert(collect_rois.end(),rois.begin(),rois.end());
      }
      std::cout << "累积目标数：" << collect_rois.size() << std::endl;
      keep_rois = do_merge_nms(collect_rois,0.1,filter_thresh);
      std::cout << "检测到目标数：" << keep_rois.size() << std::endl;

      for(auto &roi:keep_rois){
       
        std::cout << "拍照目标..." << std::endl;
        std::cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;
      }

      for(auto &roi:keep_rois){        
        vehicle->advancedSensing->getMainCameraImage(mainImg);
        Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
        //进行目标检测
        // frame = yolo_detector.detect(frame,0.2);        
        
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

        // DJI::OSDK::Gimbal::SpeedData gimbalSpeed;

        // //设置追踪器        
        // tracker = new KCFTracker(true, true, false, false);
        // tracker->init(roi, frame);
        // tu.startTracker();
        // std::cout << "dx=" << dx << " dy=" << dy << std::endl;
        // while(abs(dx) > 10 && abs(dy) > 10){
        //   vehicle->advancedSensing->getMainCameraImage(mainImg);
        //   frame = Mat(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
        //   roi = tracker->update(frame);
        //   // send gimbal speed command
        //   dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
        //   dy = (int)(roi.y + roi.height/2 - mainImg.height/2);
        //   cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
        //   if(roi.width != 0)
        //   {
        //     cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        //     cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
        //             Point(roi.x + roi.width/2, roi.y + roi.height/2),
        //             cv::Scalar(0,255,255));
        //   }
        //   cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
        //   cvtColor(frame, frame, COLOR_RGB2BGR);

        //   cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
        //   cvtColor(frame, frame, COLOR_RGB2BGR);
        //   sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
        //   putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        //   putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        //   cv::imshow(winName, frame);
        //   cv::waitKey(1);  

        //   std::cout << "调整云台" << std::endl;
        //   std::cout << "dx=" << dx << "dy=" << dy << std::endl;
        //   yawRate   = dx;
        //   pitchRate = -dy;
        //   gimbalSpeed.roll     = 0;
        //   gimbalSpeed.pitch    = pitchRate;
        //   gimbalSpeed.yaw      = yawRate;
        //   gimbalSpeed.gimbal_control_authority = 1;

        //   vehicle->gimbal->setSpeed(&gimbalSpeed);
          
        // }
        // tu.stopTracker();

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
        sleep(1);
        
        // Take picture
        std::cout << "Ensure SD card is present.\n";
        std::cout << "Taking picture..\n";
        vehicle->camera->shootPhoto();
        sleep(1);

        std::cout << "Check DJI GO App or SD card for a new picture.\n";

        std::cout << "Setting new Gimbal rotation angle to initial using absolute "
                    "control:\n";
        // Get current gimbal data to calc precision error in post processing
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
          currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
          currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
          currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
        }
        else
        {
          currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
          currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
          currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
        }
        std::cout << "回滚前。。。" << std::endl;
        displayResult(&currentAngle);
        
        // 云台回滚到开始角度
        gimbal = GimbalContainer((initialAngle.roll-currentAngle.roll)*10, (initialAngle.pitch-currentAngle.pitch)*10, (initialAngle.yaw-currentAngle.yaw)*10, 10, 0, initialAngle);        
        doSetGimbalAngle(vehicle, &gimbal);
        std::cout << "回滚后。。。" << std::endl;
        // Get current gimbal data to calc precision error in post processing
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
          initialAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
          initialAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
          initialAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
        }
        else
        {
          initialAngle.roll  = vehicle->broadcast->getGimbal().roll;
          initialAngle.pitch = vehicle->broadcast->getGimbal().pitch;
          initialAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
        }
        displayResult(&initialAngle);
        // cv::waitKey(0);

        // vehicle->advancedSensing->getMainCameraImage(mainImg);
        // frame = Mat(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
        // cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
        // if(roi.width != 0)
        // {
        //   cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        //   cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
        //           Point(roi.x + roi.width/2, roi.y + roi.height/2),
        //           cv::Scalar(0,255,255));
        // }
        // for(auto &kroi:keep_rois){
        //     cv::Rect kroi2(kroi.x - dx,kroi.x + dy,kroi.width,kroi.height);
        //     cv::rectangle(frame, kroi, cv::Scalar(0,0,255), 1, 8, 0 );
        // }
        // cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
        // cvtColor(frame, frame, COLOR_RGB2BGR);
        // sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
        // putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        // putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
        // cv::imshow(winName, frame);
        // cv::waitKey(1);  
      

      // for(auto &roi:keep_rois){
      //   vehicle->advancedSensing->getMainCameraImage(mainImg);
      //   Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
      //   //进行目标检测
      //   // frame = yolo_detector.detect(frame,0.2);
      //   std::cout << "执行拍照..." << std::endl;
      //   std::cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;

      //   //设置追踪器
        
      //   tracker = new KCFTracker(true, true, false, false);
      //   tracker->init(roi, frame);
      //   tu.startTracker();

      //   dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
      //   dy = (int)(roi.y + roi.height/2 - mainImg.height/2);
      //   cout << "dx=" << dx << " dy=" << dy << endl;
      //   if(abs(dx) > 10 && abs(dy) > 10){
      //     cout << "获取视频。。。" << endl;
      //     vehicle->advancedSensing->getMainCameraImage(mainImg);
      //     Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
      //     //调整云台
      //     cout << "调整云台。。。" << endl;
      //     roi = tracker->update(frame);
      //     trackerFinishTime = std::chrono::high_resolution_clock::now();
      //     trackerTimeDiff = trackerFinishTime - trackerStartTime;
      //     sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);
      //     // send gimbal speed command
      //     dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
      //     dy = (int)(roi.y + roi.height/2 - mainImg.height/2);

      //     yawRate   = dx;
      //     pitchRate = -dy;

      //     if(abs(dx) < 10)
      //     {
      //       yawRate = 0;
      //     }

      //     if(abs(dy) < 10)
      //     {
      //       pitchRate = 0;
      //     }

      //     DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
      //     gimbalSpeed.roll     = 0;
      //     gimbalSpeed.pitch    = pitchRate;
      //     gimbalSpeed.yaw      = yawRate;
      //     gimbalSpeed.gimbal_control_authority = 1;

      //     vehicle->gimbal->setSpeed(&gimbalSpeed);

      //     cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
      //     if(roi.width != 0)
      //     {
      //       cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

      //       cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
      //               Point(roi.x + roi.width/2, roi.y + roi.height/2),
      //               cv::Scalar(0,255,255));
      //     }

      //     cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      //     cvtColor(frame, frame, COLOR_RGB2BGR);
      //     sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      //     putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      //     putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      //     cv::imshow(winName, frame);
      //     cv::waitKey(1);  
      //   }
        
        // // Take picture
        // std::cout << "Ensure SD card is present.\n";
        // std::cout << "Taking picture..\n";
        // vehicle->camera->shootPhoto();
        // // cv::waitKey(0);
        // std::cout << "Check DJI GO App or SD card for a new picture.\n";

        // std::cout << "Setting new Gimbal rotation angle to initial using absolute "
        //             "control:\n";
        // // Get current gimbal data to calc precision error in post processing
        // if (!vehicle->isM100() && !vehicle->isLegacyM600())
        // {
        //   currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
        //   currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
        //   currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
        // }
        // else
        // {
        //   currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
        //   currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
        //   currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
        // }
        // std::cout << "回滚前。。。" << std::endl;
        // displayResult(&currentAngle);
        
        // // 云台回滚到开始角度
        
        // doSetGimbalAngle(vehicle, &gimbal);
        // std::cout << "回滚后。。。" << std::endl;
        // // Get current gimbal data to calc precision error in post processing
        // if (!vehicle->isM100() && !vehicle->isLegacyM600())
        // {
        //   currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
        //   currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
        //   currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
        // }
        // else
        // {
        //   currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
        //   currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
        //   currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
        // }
        // displayResult(&currentAngle);

      }
      cv::waitKey(0);  

      break;

      // if(count % 30*5 == 0){
      //   cout << "执行检测" << endl;

      //   vehicle->advancedSensing->getMainCameraImage(mainImg);
      //   Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);

      //   //进行目标检测
      //   // frame = yolo_detector.detect(frame,0.2);
      //   vector<Rect> rois = yolo_detector.detect_bboxes(frame,frame,0.2);
      //   if(!rois.empty()){
      //     roi = rois[0];
      //     cout <<"roi: " << roi << endl;
      //     tracker = new KCFTracker(true, true, false, false);
      //     tracker->init(roi, frame);
      //     tu.startTracker();
      //   }
      //   else{
      //     tracker = NULL;
      //     tu.stopTracker();
      //     roi = Rect(0,0,0,0);
      //   }

      // }
      
      // vehicle->advancedSensing->getMainCameraImage(mainImg);
      // Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);


      // //进行目标检测
      // frame = yolo_detector.detect(frame,0.2);
      // // vector<Rect> rois = yolo_detector.detect_bboxes(frame,&frame,0.2);

      // switch(tu.getState())
      // {
      // case TrackingUtility::STATE_IDLE:
      //   roi = tu.getROI();
      //   sprintf(message2, "Please select ROI and press g");
      //   break;

      // case TrackingUtility::STATE_INIT:
      //   cout << "g pressed, initialize tracker" << endl;
      //   sprintf(message2, "g pressed, initialize tracker");
      //   roi = tu.getROI();
      //   tracker = new KCFTracker(true, true, false, false);
      //   tracker->init(roi, frame);
      //   tu.startTracker();
      //   break;

      // case TrackingUtility::STATE_ONGOING:
      //   trackerStartTime  = std::chrono::high_resolution_clock::now();
      //   roi = tracker->update(frame);
      //   trackerFinishTime = std::chrono::high_resolution_clock::now();
      //   trackerTimeDiff = trackerFinishTime - trackerStartTime;
      //   sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);

      //   // send gimbal speed command
      //   dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
      //   dy = (int)(roi.y + roi.height/2 - mainImg.height/2);

      //   yawRate   = dx;
      //   pitchRate = -dy;

      //   if(abs(dx) < 10)
      //   {
      //     yawRate = 0;
      //   }

      //   if(abs(dy) < 10)
      //   {
      //     pitchRate = 0;
      //   }

      //   DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
      //   gimbalSpeed.roll     = 0;
      //   gimbalSpeed.pitch    = pitchRate;
      //   gimbalSpeed.yaw      = yawRate;
      //   gimbalSpeed.gimbal_control_authority = 1;

      //   vehicle->gimbal->setSpeed(&gimbalSpeed);

      //   // Take picture
      //   std::cout << "Ensure SD card is present.\n";
      //   std::cout << "Taking picture..\n";
      //   vehicle->camera->shootPhoto();
      //   usleep(1000);
      //   std::cout << "Check DJI GO App or SD card for a new picture.\n";

      //   std::cout << "Setting new Gimbal rotation angle to [0,-50, 0] using absolute "
      //               "control:\n";

      //   break;

      // case TrackingUtility::STATE_STOP:
      //   cout << "s pressed, stop tracker" << endl;
      //   sprintf(message2, "s pressed, stop tracker");
      //   delete tracker;
      //   tracker = NULL;
      //   tu.stopTracker();
      //   roi = tu.getROI();
      //   break;

      // default:
      //   break;
      // }

      // dx = roi.x + roi.width/2  - mainImg.width/2;
      // dy = roi.y + roi.height/2 - mainImg.height/2;

      // cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
      // if(roi.width != 0)
      // {
      //   cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

      //   cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
      //            Point(roi.x + roi.width/2, roi.y + roi.height/2),
      //            cv::Scalar(0,255,255));
      // }

      // cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      // cvtColor(frame, frame, COLOR_RGB2BGR);
      // sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      // putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      // putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      // cv::imshow(winName, frame);
      // cout << "sending stream..." << endl;
      // process_frame(frame, proc_frame);
      // stream_frame(streamer, proc_frame);
      // count ++;

    }
  }

  vehicle->advancedSensing->stopMainCameraStream();

  if(tracker)
  {
    delete tracker;
  }

  return 0;
}



// int main(int argc, char** argv)
// {
//   //Setup detector
//   string  names_file = "/home/nvidia/darknet/data/voc.names";
//   string  cfg_file = "/home/nvidia/darknet/cfg/yolov3-tiny-prn.cfg";
//   string  weights_file = "/home/nvidia/darknet/backup/yolov3-tiny-prn_best.weights";
//   string video_file = "/home/breeze/MyCode/darknet_streaming/build/DJI_0318.mp4";

//   //初始化检测器
//   Mat result_frame;
//   YoloObjectDetector yolo_detector(names_file,cfg_file,weights_file);

//   VideoCapture cap = VideoCapture(0);
//   Mat frame;

//   while(1)
//   {
//     if(cap.isOpened())
//     {
      
//       cout << "reading frame..." << endl;
//       cap >> frame;
      
//       // 累积多帧图片的检测结果
//       vector<Rect> collect_rois,keep_rois;
//       int accumulated_frame_count = 20;
//       int filter_thresh = (int)(accumulated_frame_count * 0.6);
//       for(int i=0;i<accumulated_frame_count;i++){
//         cap >> frame;
//         //进行目标检测
//         vector<Rect> rois = yolo_detector.detect_bboxes(frame,frame,0.2);
//         cv::imshow("winName", frame);
//         cv::waitKey(1);
//         collect_rois.insert(collect_rois.end(),rois.begin(),rois.end());
//       }
//       std::cout << "累积目标数：" << collect_rois.size() << std::endl;
//       try{
//         keep_rois = do_merge_nms(collect_rois,0.3,filter_thresh);

//       }
//       catch(char *str)
//       {
//         cout << "Exception caught: " << str << endl;
//       }
      
//       std::cout << "检测到目标数：" << keep_rois.size() << std::endl;
      
//       cv::imshow("winName", frame);
//       cv::waitKey(1);

//     }
//   }

//   return 0;
// }
