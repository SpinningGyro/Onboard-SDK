/*
 * YoloObjectDetector.hpp
 *
 *  Created on: 2020.06.27
 *      Author: Breeze Ng
 *   Institute: Beijing YuHang
 */

#pragma once
#ifndef YOLOOBJECTDETECTION_HPP
#define YOLOOBJECTDETECTION_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp"    // imported functions from DLL

namespace detector
{

// template<typename T>
// class send_one_replaceable_object_t {
//     const bool sync;
//     std::atomic<T *> a_ptr;
// public:

//     void send(T const& _obj) {
//         T *new_ptr = new T;
//         *new_ptr = _obj;
//         if (sync) {
//             while (a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
//         }
//         std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
//     }

//     T receive() {
//         std::unique_ptr<T> ptr;
//         do {
//             while(!a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
//             ptr.reset(a_ptr.exchange(NULL));
//         } while (!ptr);
//         T obj = *ptr;
//         return obj;
//     }

//     bool is_object_present() {
//         return (a_ptr.load() != NULL);
//     }

//     send_one_replaceable_object_t(bool _sync) : sync(_sync), a_ptr(NULL)
//     {}
// };

typedef struct{
   float x, y, w, h; //x,y 图像中心, w,h 图像宽和高
   int prob; // 是否保留
} box;

float overlap(float x1, float w1, float x2, float w2);
float box_intersection(box a, box b);
float box_union(box a, box b);
float box_iou(box a, box b);
box merge_box(box a, box b);
box rect2box(cv::Rect rect);
cv::Rect box2rect(box b);
std::vector<cv::Rect> merge_boxes(std::vector<std::vector<box>> group_boxes,int thresh);
std::vector<cv::Rect> do_merge_nms(std::vector<cv::Rect> boxes,float thresh,int filter_thresh);


struct detection_data_t {
    cv::Mat cap_frame;
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> result_vec;
    cv::Mat draw_frame;
    bool new_detection;
    uint64_t frame_id;
    bool exit_flag;
    cv::Mat zed_cloud;
    std::queue<cv::Mat> track_optflow_queue;
    detection_data_t() : exit_flag(false), new_detection(false) {}
};

class YoloObjectDetector
{
    std::string  names_file;
    std::string  cfg_file;
    std::string  weights_file;
    std::vector<std::string> obj_names;
    detection_data_t detection_data;
    Detector *detector;
    
public:
    YoloObjectDetector(std::string src_names_file,std::string src_cfg_file,std::string src_weights_file);
    ~YoloObjectDetector();
    std::vector<std::string> objects_names_from_file(std::string const filename);
    void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
                    int current_det_fps = -1, int current_cap_fps = -1);
    cv::Mat detect(cv::Mat frame,float thresh);
    std::vector<cv::Rect> get_rois(cv::Mat mat_img, std::vector<bbox_t> result_vec);
    std::vector<cv::Rect> detect_bboxes(cv::Mat frame,cv::Mat &draw_frame,float thresh);

};

} // namespace detector
#endif