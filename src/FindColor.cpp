
#include "ColorDetector.h"


int main()
{
    ColorDetector theColorDetector = ColorDetector();

    cv::VideoCapture capture(0);
    while (true)
    {
        cv::Mat frame;
        capture >> frame;

        cv::Rect Left(0, 0, frame.cols/2, frame.rows);
        cv::Mat frame_l = frame(Left);


        cv::Mat inputImage = frame_l.clone();
        
        frame.clone();
        int linewidth = 20;

        cv::Point2i pixel_red = theColorDetector.calculate_center(theColorDetector.distinguish_color(inputImage, theColorDetector.RED));
        std::cout<<"red: "<<pixel_red<<std::endl;
        cv::circle(inputImage, pixel_red, linewidth, cv::Scalar(0,0,255), -1, 8, 0);
        cv::circle(inputImage, pixel_red, linewidth, cv::Scalar(0,0,0), 3, 8, 0);

        cv::Point2i pixel_green = theColorDetector.calculate_center(theColorDetector.distinguish_color(inputImage, theColorDetector.GREEN));
        std::cout<<"green: "<<pixel_green<<std::endl;
        cv::circle(inputImage, pixel_green, linewidth, cv::Scalar(0,255,0), -1, 8, 0);
        cv::circle(inputImage, pixel_green, linewidth, cv::Scalar(0,0,0), 3, 8, 0);

        cv::namedWindow("camera_l", 0);
        cvResizeWindow("camera_l", 960, 960/frame_l.cols*frame_l.rows);
        cv::moveWindow("camera_l",500,250);
        cv::imshow("camera_l", frame_l);

        cv::namedWindow("result", 0);
        cvResizeWindow("result", 960, 960/inputImage.cols*inputImage.rows);
        cv::moveWindow("result",1500,250);
        cv::imshow("result", inputImage);
        cv::waitKey(1);


    }


    return 0;
}