#ifndef __COLOR_DETECTOR_H__
#define __COLOR_DETECTOR_H__

#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>



class ColorDetector
{
public:
    enum color_type {RED=0, GREEN, BLUE};
public:
    ColorDetector();
    cv::Point2f position_to_pixel(const cv::Mat &H, const cv::Mat &position);
    cv::Mat pixel_to_position(const cv::Mat &H, const cv::Point2f &pixel);
    cv::Mat distinguish_color(const cv::Mat &inputImage, const color_type &type);
    cv::Point2i calculate_center(const cv::Mat &inputImage);
    cv::Mat find_H_test();
    cv::Mat find_H();

};



#endif