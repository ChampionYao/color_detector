
#include "ColorDetector.h"


int main()
{
    ColorDetector theColorDetector = ColorDetector();


    cv::Mat image = cv::imread("../files/780.jpg", 1);

    // The positions is in meters and are obtained with the homography matrix H.
    cv::Mat position = (cv::Mat_ <double>(3, 1) << 
                            8.4568443e+00, 
                            3.5880664e+00,
                            1
                            );

    cv::Mat H = (cv::Mat_ <double>(3, 3) << 
                    2.8128700e-02, 2.0091900e-03, -4.6693600e+00,
                    8.0625700e-04, 2.5195500e-02, -5.0608800e+00,
                    3.4555400e-04, 9.2512200e-05, 4.6255300e-01
                    );

    cv::Point2f p = theColorDetector.position_to_pixel(H, position);
    p = cv::Point2f(p.y, p.x);
    cv::circle(image, p, 10, cv::Scalar(0,0,255), 2);
    cv::imshow("frame", image);
    cv::waitKey(0);

    std::cout<<theColorDetector.pixel_to_position(H, theColorDetector.position_to_pixel(H, position))<<std::endl;


    theColorDetector.find_H_test();


    return 0;
}