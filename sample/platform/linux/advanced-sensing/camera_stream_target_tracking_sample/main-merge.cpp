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
#include <fstream>
#include <sstream>
#include <ctime>
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

#include "communicator.hpp"

#define DEG2RAD 0.01745329252

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;
using namespace cv;
using namespace streamer;
using namespace detector;

void print_log(std::string log){
   // 打印日志
  time_t now = time(0);
  char* dt = ctime(&now);
  ofstream outfile;
  outfile.open("Log.txt", ios::out | ios::app);
  outfile << dt <<" ";
  outfile << log << endl;
  outfile.close();
}


void process_frame(const cv::Mat &in, cv::Mat &out)
{
    in.copyTo(out);
}

void stream_frame(Streamer &streamer, const cv::Mat &image)
{
    streamer.stream_frame(image.data);
}

// 初始化推流器
int startStreamer(Vehicle* vehicle, Streamer &streamer) {

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
  // int timeout = 1;
  // if(!setUpSubscription(vehicle,timeout)){
  //   return -4;
  // }
  return 1;
}

bool run_take_photo(Vehicle* vehicle, CameraManagerSyncSample *cam, GimbalManagerSyncSample *gim, YoloObjectDetector* bj_yolo_detector) {
  CameraRGBImage mainImg;
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);

  cv::namedWindow(winName,1);

  int count = 0;
  int dx = 0;
  int dy = 0;
  int yawRate = 0;
  int pitchRate = 0;

  // 获取云台初始角度

  cout << "获取云台初始角度" << endl;
  GimbalSingleData initialGimbalData = gim->getGimbalData(PAYLOAD_INDEX_0);
    DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
            initialGimbalData.pitch,
            initialGimbalData.roll,
            initialGimbalData.yaw); 
  // try{
  //   initialGimbalData = gim->getGimbalData(PAYLOAD_INDEX_0);
  //   DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
  //           initialGimbalData.pitch,
  //           initialGimbalData.roll,
  //           initialGimbalData.yaw); 
  // }catch(char *str){
  //   DERROR("获取云台初始角度错误：%s",str);
  //   initialGimbalData.pitch = 0;
  //   initialGimbalData.roll = 0;
  //   initialGimbalData.yaw = 0;
  // }      

  // 中心点对焦
  cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);
  usleep(1 * 1000 * 1000);

  // 累积多帧图片的检测结果
  vector<Rect> collect_rois, keep_rois;
  int accumulated_frame_count = 30;

  int filter_thresh = (int)(accumulated_frame_count * 0.6);
  // int filter_thresh = (int)(accumulated_frame_count);

  for(int i=0; i<accumulated_frame_count; i++) {
    vehicle->advancedSensing->getMainCameraImage(mainImg);
    Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
    //进行目标检测
    vector<Rect> rois = bj_yolo_detector->detect_bboxes(frame,frame,0.2);
    cvtColor(frame, frame, COLOR_RGB2BGR);
    cv::imshow(winName, frame);
    cv::waitKey(1);
    collect_rois.insert(collect_rois.end(),rois.begin(),rois.end());
  }
  std::cout << "累积目标数：" << collect_rois.size() << std::endl;
  keep_rois = do_merge_nms(collect_rois, 0.1, filter_thresh);
  std::cout << "检测到目标数：" << keep_rois.size() << std::endl;

 // 打印日志
  ostringstream oss;
  oss << "累积目标数：" << collect_rois.size();
  print_log(oss.str());
  oss << "检测到目标数：" << keep_rois.size();
  print_log(oss.str());

  // 如果没有检测到目标
  if(keep_rois.size() == 0) {
    cam->setZoomSyncSample(PAYLOAD_INDEX_0, 5); // 放大5倍
    usleep(2 * 1000 * 1000);
    cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);  // 中心点对焦
    usleep(3 * 1000 *1000);
    cam->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);  // 拍照
    usleep(1 * 1000 * 1000);
    cam->setZoomSyncSample(PAYLOAD_INDEX_0, 2);  // 变回原焦距（两倍）
    usleep(1 * 1000 * 1000);
    std::cout << "没有检测到目标,执行完成";

    // 打印日志
    print_log("没有检测到目标,执行完成");

    return true;
  }

  Rect roiCenter;
  roiCenter.x = 0;
  roiCenter.y = 0;
  float lastDistance = 100 * 1000 * 1000;
  for(auto &kroi:keep_rois) {

    float idx = kroi.x + kroi.width *0.5 - mainImg.width * 0.5;
    float idy = kroi.y + kroi.height *0.5 - mainImg.height * 0.5;
    float currentDistance = pow((pow(idx, 2) + pow(idy, 2)), 0.5);

    // 打印日志
    oss << "识别框中心点坐标：" << kroi.x + kroi.width * 0.5 << kroi.y + kroi.height * 0.5;
    print_log(oss.str());
    oss << "视频流宽高：" <<  mainImg.width << mainImg.height;
    print_log(oss.str());
    oss << "识别框距离" << currentDistance;
    print_log(oss.str());

    if (currentDistance < lastDistance) {
      lastDistance = currentDistance;
      roiCenter = kroi;
    }
  }


  // 只对距中心点最近的单个目标框做动作
  keep_rois.clear();
  keep_rois.push_back(roiCenter);

  for(int c = 0; c < keep_rois.size(); c++) {
    if(c > 1)
        break;
    roi = keep_rois[c];

    // 打印日志
    oss << "目标框中心点坐标" << roi.x + roi.width * 0.5 << roi.y + roi.height * 0.5;
    print_log(oss.str());

   //for(auto &roi:keep_rois) {
    vehicle->advancedSensing->getMainCameraImage(mainImg);
    Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3); 
    
    std::cout << "执行拍照..." << std::endl;
    std::cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;

    for(auto &kroi:keep_rois) {
        cv::Rect kroi2(kroi.x - dx, kroi.y + dy, kroi.width, kroi.height); // kroi.y + dy (zhang)
        cv::rectangle(frame, kroi, cv::Scalar(0,0,255), 1, 8, 0 );
    }

    cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
    if(roi.width != 0) {
      cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

      cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
              Point(roi.x + roi.width/2, roi.y + roi.height/2),
              cv::Scalar(0,255,255));
    }
    cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
    cvtColor(frame, frame, COLOR_RGB2BGR);
    
    // send gimbal speed command
    dx = (int)(roi.x + roi.width / 2.  - mainImg.width / 2.);
    dy = (int)(roi.y + roi.height / 2. - mainImg.height / 2.);
    sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
    putText(frame, message1, Point2f(20, 30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
    putText(frame, message2, Point2f(20, 60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
    cv::imshow(winName, frame);
    cv::waitKey(1);

    // int ratio_x = round(2 * 0.6 * mainImg.width / roi.width);
    // int ratio_y = round(2 * 0.6 * mainImg.height / roi.height);
    int ratio = 5;
    // if (ratio_x > ratio_y){
    //   ratio = ratio_y;
    // }
    // else{
    //   ratio = ratio_x;
    // }

    // send gimbal speed command

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

    usleep(1 * 1000 * 1000);

    // float w_scale =  (mainImg.width - roi.width) / (mainImg.width + 500) * 50;
    // float h_scale =  (mainImg.width - roi.height) / (mainImg.height + 500) * 50;
    // float scale = w_scale < h_scale ? w_scale:h_scale;

    // std::cout << "图像大小：" << mainImg.width << " " << mainImg.height << endl;
    // std::cout << "变焦倍数：" << scale << std::endl;
    // if(scale > 50){
    //   scale = 50;
    // }
    // if(scale < 2){
    //   scale = 2;
    // }
    // cam->setZoomSyncSample(PAYLOAD_INDEX_0, 10);

    cam->setZoomSyncSample(PAYLOAD_INDEX_0, ratio);
    usleep(2 * 1000 * 1000);
    cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);
    usleep(3 * 1000 * 1000);
    cam->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);
    usleep(1 * 1000 * 1000);
    cam->setZoomSyncSample(PAYLOAD_INDEX_0, 2);  // 相机回到原焦距
    usleep(1 * 1000 * 1000);
    // 云台转回初始位置
    GimbalModule::Rotation rotation;
    rotation.roll = 0.0f;
    rotation.pitch = initialGimbalData.pitch;
    rotation.yaw = initialGimbalData.yaw;
    rotation.rotationMode = 0;
    rotation.time = 0.5;
    gim->rotateSyncSample(PAYLOAD_INDEX_0, rotation);
    usleep(1 * 1000 * 1000);
  }
  // cam->setZoomSyncSample(PAYLOAD_INDEX_0, 2);
  return true;
}

// 如果是绝缘子本体，不做检测，只放大
bool run_focus(Vehicle* vehicle, CameraManagerSyncSample *cam, GimbalManagerSyncSample *gim) {
  cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);  // 中心点对焦
  usleep(1 * 1000 * 1000);
  cam->setZoomSyncSample(PAYLOAD_INDEX_0, 5); // 放大5倍
  usleep(2 * 1000 * 1000);
  cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);  // 中心点对焦
  usleep(3 * 1000 *1000);
  cam->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);  // 拍照
  usleep(1 * 1000 * 1000);
  cam->setZoomSyncSample(PAYLOAD_INDEX_0, 2);  // 变回原焦距（两倍）
  usleep(1 * 1000 * 1000);
  std::cout << "绝缘子本体，执行完成";

  // 打印日志
  print_log("绝缘子本体，执行完成");

  return true;
}

float last_h = 720;

// 导地线巡检
bool run_ddx(Vehicle* vehicle,CameraManagerSyncSample *cam, GimbalManagerSyncSample *gim, YoloObjectDetector* dx_yolo_detector) {
  CameraRGBImage mainImg;
  int count = 0;
  int dx = 0;
  int dy = 0;
  int yawRate = 0;
  int pitchRate = 0;
  Rect roi(0,0,0,0);

  // 获取云台初始角度
  cout << "获取云台初始角度" << endl;
  GimbalSingleData initialGimbalData = gim->getGimbalData(PAYLOAD_INDEX_0);
    DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
            initialGimbalData.pitch,
            initialGimbalData.roll,
            initialGimbalData.yaw);

  // // 中心对焦
  // cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);  
  // usleep(1 * 1000 * 1000);
  
  vehicle->advancedSensing->getMainCameraImage(mainImg);
  Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
  //进行目标检测
  vector<Rect> rois = dx_yolo_detector->detect_bboxes(frame,frame,0.2);
  cv::imshow("winName",frame);
  cv::waitKey(1);

  // H20相机视频像素为3840*2160或1920*1080，原720可能有误
  if (last_h ==720) {
   last_h = mainImg.height * 0.5;
  }
  
  if(rois.size() > 0) {
    Rect min_roi;

    // // 取最下面的目标框
    // float min_dist = -1.0;
    // for(int k=0; k < rois.size(); k++) {
    //   float ds = rois[k].y + rois[k].height * 0.5;
    //   if(ds > min_dist) {
    //     min_dist = ds;
    //     min_roi = rois[k];
    //   }
    // }

    // 取离上一帧最近的框
    float min_dist = 1000 * 1000 * 1000;
    for(int k=0; k < rois.size(); k++) {
      float ds = rois[k].y + rois[k].height * 0.5;

      // 打印日志
      ostringstream oss;
      oss << "识别框中心点Y坐标：" << rois[k].y + rois[k].height * 0.5;
      print_log(oss.str());
      oss << "视频流高：" << mainImg.height;
      print_log(oss.str());
      oss << "识别框Y方向距离：" << ds;
      print_log(oss.str());

      if(abs(ds - last_h) < min_dist) {
        min_dist = abs(ds - last_h);
        min_roi = rois[k];
      }
    }

    vehicle->advancedSensing->getMainCameraImage(mainImg);
    Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
    // roi = rois[0];
    roi = min_roi;

    // 打印日志
    oss << "目标框中心点Y坐标：" << roi.y;
    print_log(oss.str());

    last_h = roi.y + roi.height / 2.;
    // send gimbal speed command
    dx = (int)(roi.x + roi.width / 2.  - mainImg.width / 2.);
    dy = (int)(roi.y + roi.height / 2. - mainImg.height / 2.);
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
    gimbalSpeed.yaw      = 0;//yaw;
    gimbalSpeed.gimbal_control_authority = 1;

    vehicle->gimbal->setSpeed(&gimbalSpeed);
    usleep(0.5 * 1000 * 1000);
  }
  // cam->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.5, 0.5);
  // usleep(1 * 1000 * 1000);
  cam->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);
  usleep(1 * 1000 * 1000);
  return true;
}

int main(int argc, char** argv)
{
  //初始化部件检测器
  string  bj_names_file = "/home/nvidia/darknet/data/voc.names";
  string  bj_cfg_file = "/home/nvidia/darknet/cfg/yolov3-tiny-prn.cfg";
  string  bj_weights_file = "/home/nvidia/darknet/backup/yolov3-tiny-prn_best.weights";
  YoloObjectDetector* bj_yolo_detector = new YoloObjectDetector(bj_names_file,bj_cfg_file,bj_weights_file);

  //初始化导线检测器
  string  dx_names_file = "/home/nvidia/darknet/backup/daodixian/ddx.names";
  string  dx_cfg_file = "/home/nvidia/darknet/backup/daodixian/yolov4.cfg";
  string  dx_weights_file = "/home/nvidia/darknet/backup/daodixian/yolov4_ddx.weights";
  YoloObjectDetector* dx_yolo_detector = new YoloObjectDetector(dx_names_file,dx_cfg_file,dx_weights_file);

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
    if(ret){
      break;
    }else{
      sleep(5);
    }
  }
  DSTATUS("启动 OSDK 环境 成功");

  // 开启通讯
  Communicator comm(vehicle);
  int retCom = comm.threadRun();
  if(!retCom){
    DERROR("通讯建立失败!\n");

    // 打印日志
    print_log("通讯建立失败!");

  }
  
   // 获取控制权时无法控制云台（3.9),(4.0未测试)
   // Initialize variables
   int functionTimeout = 1;
   // Obtain Control Authority
   vehicle->control->obtainCtrlAuthority(functionTimeout);

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

  while(true)
  {    
    // comm.isStart = 1;
    // comm.isResume = 1;
    if(vehicle->advancedSensing->newMainCameraImageReady() && comm.isStart==1) {
      run_take_photo(vehicle, cam, gim, bj_yolo_detector);
      comm.isStart = 0;

      // 打印日志
      print_log("run_take_photo");

    }
    if(vehicle->advancedSensing->newMainCameraImageReady() && comm.isFocus==1) {
      run_focus(vehicle, cam, gim);
      comm.isFocus = 0;

      // 打印日志
      print_log("run_focus");

    }
    if(vehicle->advancedSensing->newMainCameraImageReady() && comm.isResume == 1) {
      if(comm.isStop==1){
        comm.isResume = 0;
        comm.isStop = 0;
      }
      run_ddx(vehicle, cam, gim, dx_yolo_detector);

      // 打印日志
      print_log("run_ddx");

    }
  }

  vehicle->advancedSensing->stopMainCameraStream();

  return 0;
}
