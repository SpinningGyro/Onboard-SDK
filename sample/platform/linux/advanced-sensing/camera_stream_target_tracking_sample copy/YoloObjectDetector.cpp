#include <string>
#include <vector>
#include <fstream>
#include<algorithm>

#include "YoloObjectDetector.hpp"
#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp"
namespace detector
{

float overlap(float x1, float w1, float x2, float w2)
{
    float l1 = x1 - w1/2;
    float l2 = x2 - w2/2;
    float left = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1/2;
    float r2 = x2 + w2/2;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}

float box_intersection(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    if(w < 0 || h < 0) return 0;
    float area = w*h;
    return area;
}

float box_union(box a, box b)
{
    float i = box_intersection(a, b);
    float u = a.w*a.h + b.w*b.h - i;
    return u;
}

float box_iou(box a, box b)
{
    //return box_intersection(a, b)/box_union(a, b);

    float I = box_intersection(a, b);
    float U = box_union(a, b);
    if (I == 0 || U == 0) {
        return 0;
    }
    return I / U;
}

box merge_box(box a, box b){
    box c;
    float xmin = (a.x - a.w/2) < (b.x - b.w/2) ? (a.x - a.w/2): (b.x - b.w/2);
    float ymin = (a.y - a.h/2) < (b.y - b.h/2) ? (a.y - a.h/2): (b.y - b.h/2);
    float xmax = (a.x + a.w/2) > (b.x + b.w/2) ? (a.x + a.w/2): (b.x + b.w/2);
    float ymax = (a.y + a.h/2) > (b.y + b.h/2) ? (a.y + a.h/2): (b.y + b.h/2);
    c.w = xmax - xmin;
    c.h = ymax - ymin;
    c.x = xmin + c.w/2;
    c.y = ymin + c.h/2;
  
    if(c.x < 0)
        c.x = 0;
    if(c.y < 0)
        c.y = 0;
    return c;
}

box rect2box(cv::Rect rect){
    box b;
    b.x = rect.x + rect.width/2;
    b.y = rect.y + rect.height/2;
    b.w = rect.width;
    b.h = rect.height;
    b.prob = 1;
    return b;
}

cv::Rect box2rect(box b){
    int x = (b.x - b.w/2) > 0 ? (b.x - b.w/2):0;
    int y = (b.y - b.h/2) > 0 ? (b.y - b.h/2):0;
    return cv::Rect(x,y,b.w,b.h);
}

std::vector<cv::Rect> merge_boxes(std::vector<std::vector<box>> group_boxes,int thresh){

    int i;
    std::vector<cv::Rect> m_boxes;
    for(auto &boxes:group_boxes){
        int b_size = boxes.size();
        if(b_size < thresh)
            continue;
        box result_box = boxes[0];
        for(i=1; i<b_size; i++){
            result_box = merge_box(result_box,boxes[i]);
        }
        m_boxes.push_back(box2rect(result_box));
    }
    return m_boxes;
}

std::vector<cv::Rect> do_merge_nms(std::vector<cv::Rect> boxes,float thresh,int filter_thresh){
    int i,j;
    std::vector<std::vector<box>> group_boxes;
    int total = boxes.size();
    std::vector<int> skips;
    for(i=0; i< total-1; i++){
        std::vector<box> g_boxes;
        box b1 = rect2box(boxes[i]);
        g_boxes.push_back(b1);
        if (std::find(skips.begin(), skips.end(), i) != skips.end()){
            continue;
        }                        
        
        for(j=i+1; j<total; j++){            
            box b2 = rect2box(boxes[j]);
            if(box_iou(b1,b2) > thresh){
                skips.push_back(j);
                g_boxes.push_back(b2);                
            }                
        }
        group_boxes.push_back(g_boxes);
    }  
    return merge_boxes(group_boxes,filter_thresh);
}

YoloObjectDetector::YoloObjectDetector(std::string src_names_file,std::string src_cfg_file,std::string src_weights_file){
    names_file = src_names_file;
    cfg_file = src_cfg_file;
    weights_file = src_weights_file;
    obj_names = objects_names_from_file(names_file);     
    detector = new Detector(cfg_file, weights_file);

}
YoloObjectDetector::~YoloObjectDetector(){
    
}

//绘制检测结果
void YoloObjectDetector::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,int current_det_fps, int current_cap_fps)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);
            //max_width = std::max(max_width, 283);
            std::string coords_3d;
            if (!std::isnan(i.z_3d)) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
                coords_3d = ss.str();
                cv::Size const text_size_3d = getTextSize(ss.str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
                int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
                if (max_width_3d > max_width) max_width = max_width_3d;
            }

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            if(!coords_3d.empty()) putText(mat_img, coords_3d, cv::Point2f(i.x, i.y-1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
}

//绘制检测结果
std::vector<cv::Rect> YoloObjectDetector::get_rois(cv::Mat mat_img, std::vector<bbox_t> result_vec)
{
    std::vector<cv::Rect> rois;

    for (auto &i : result_vec) {
        
        int xmin = i.x; //(int)std::max(i.x,0);
        int ymin = i.y; //(int)std::max(i.y,0);
        int xmax = i.x + i.w; //(int)std::min(i.x + i.w, mat_img.cols);
        int ymax = i.y + i.h; //(int)std::min(i.y + i.h, mat_img.rows);
        // cv::Rect roi(xmin,ymin,xmax,ymax);
        // rois.push_back(roi);
        rois.push_back(cv::Rect(cv::Point(xmin,ymin),cv::Point(xmax,ymax)));
    }
    return rois;
}

// 获取类别名称
std::vector<std::string> YoloObjectDetector::objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
        std::cout << "object names loaded \n";
    return file_lines;
}

cv::Mat YoloObjectDetector::detect(cv::Mat frame,float thresh){       
    cv::Mat draw_frame;
    frame.copyTo(draw_frame);
    cv::Size const frame_size = frame.size();
    std::shared_ptr<image_t> det_image;
    det_image = detector->mat_to_image_resize(frame);
    std::vector<bbox_t> result_vec = detector->detect_resized(*det_image, frame_size.width, frame_size.height, thresh, true);  // true
    draw_boxes(draw_frame, result_vec, obj_names);
    return draw_frame;
    
}

std::vector<cv::Rect> YoloObjectDetector::detect_bboxes(cv::Mat frame,cv::Mat &draw_frame,float thresh){
    std::vector<cv::Rect> rois;       
    
    cv::Size const frame_size = frame.size();
    std::shared_ptr<image_t> det_image;
    det_image = detector->mat_to_image_resize(frame);
    std::vector<bbox_t> result_vec = detector->detect_resized(*det_image, frame_size.width, frame_size.height, thresh, true);  // true
    draw_boxes(draw_frame, result_vec, obj_names);
    rois = get_rois(draw_frame, result_vec);    
    return rois;    
}

} // namespace detector