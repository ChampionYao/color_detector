#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

enum color_type {RED=0, GREEN, BLUE};

cv::Point2i position_to_pixel(const cv::Mat &H, const cv::Mat &position)
{
    //normalization
    cv::Mat H_1 = H / H.at<double>(2,2);

    cv::Mat pixel = H_1.inv() * position;

    int x = pixel.at<double>(0,0)/pixel.at<double>(2,0);
    int y = pixel.at<double>(1,0)/pixel.at<double>(2,0);

    //the x, y should be exchange because of the image frame
    return cv::Point2i(y, x);
}

cv::Mat pixel_to_position(const cv::Mat &H, const cv::Point2i &pixel)
{
    cv::Mat p = (cv::Mat_ <double>(3, 1) << 
                    pixel.y, 
                    pixel.x,
                    1
                    );
    cv::Mat H_1 = H / H.at<double>(2,2);
    cv::Mat position = H_1 * p;
    position = position / position.at<double>(2,0);
    return position;

}

cv::Mat distinguish_color(const cv::Mat &inputImage, const color_type &type)
{
    CvScalar red = CvScalar(0, 0, 255);
    CvScalar green = CvScalar(0, 255, 0);
    CvScalar blue = CvScalar(255, 0, 0);
    CvScalar black = CvScalar(0, 0, 0);

    CvScalar color;
    int h_min_1, h_min_2, h_max_1, h_max_2, s_min, s_max, v_min, v_max;

    switch (type)
    {
    case 0:
        color = red;
        h_min_1 = 0;
        h_max_1 = 10;
        h_min_2 = 156;
        h_max_2 = 180;
        s_min = 23;
        s_max = 255;
        v_min = 46;
        v_max = 255;
        break;
    case 1:
        color = green;
        h_min_1 = 30;
        h_max_1 = 99;
        h_min_2 = h_min_1;
        h_max_2 = h_max_1;
        s_min = 43;
        s_max = 255;
        v_min = 46;
        v_max = 255;
        break;
    // case 2:
    //     color = blue;
    //     h_min_1 = 100;
    //     h_max_1 = 124;
    //     h_min_2 = h_min_1;
    //     h_max_2 = h_max_1;
    //     s_min = 43;
    //     s_max = 255;
    //     v_min = 46;
    //     v_max = 255;
    //     break;
    default:
        std::cout<<"undefined color"<<std::endl;
        return inputImage;
    }


    //to hsv
    IplImage temp = IplImage(inputImage);
    IplImage* image = &temp;
    IplImage* hsv_img = cvCreateImage( cvGetSize(image), 8, 3 );  
    cvCvtColor(image, hsv_img, CV_BGR2HSV);

	for (int i = 0; i < hsv_img->height; i++)
    {
        for (int j = 0; j < hsv_img->width; j++)
		{
			CvScalar hsv = cvGet2D(hsv_img, i, j);
            int h = hsv.val[0];
            int s = hsv.val[1];
            int v = hsv.val[2];
            
            if( ((h>h_min_1&&h<h_max_1) || (h>h_min_2&&h<h_max_2)) && (s>s_min && s<s_max) && (v>v_min && v<v_max) )
            {
			    cvSet2D(hsv_img, i ,j, color);
            }
            else
            {
                cvSet2D(hsv_img, i ,j, black);
            }
		}
    }
		
    cv::Mat outputImage = cv::cvarrToMat(hsv_img);

    //filter
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(8,8)); 
    cv::medianBlur(outputImage, outputImage, 5);  
    cv::morphologyEx(outputImage, outputImage, cv::MORPH_OPEN, kernel); 
    cv::morphologyEx(outputImage, outputImage, cv::MORPH_CLOSE, kernel);  
    cv::blur(outputImage, outputImage, cv::Size(5, 5));  

    cv::imwrite("../files/distinguish_color.jpg", outputImage);

    return outputImage;

}

cv::Point2i calculate_center(const cv::Mat &inputImage)
{
    //Binarization
    cv::Mat inputImage_gray;
    cvtColor(inputImage, inputImage_gray, CV_BGR2GRAY);

    int linewidth = 10;
    cv::Scalar color = cv::Scalar(255, 255, 255);
    cv::Mat frame = inputImage.clone();

    //find contours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(inputImage_gray.clone(), contours, hierarchy,CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // //calculate the center
    // cv::Moments mu = moments(contour, false);
    // cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

    // plot
    // drawContours(frame, contours, largest_contour_index, color, linewidth, 8, hierarchy, 0, cv::Point());
    // cv::circle(frame, mc, linewidth, color, -1, 8, 0);

    int min_x, min_y, max_x, max_y;
    min_x = inputImage_gray.cols;
    max_x = 0;
    min_y = inputImage_gray.rows;
    max_y = 0;

    double map_area = inputImage_gray.cols * inputImage_gray.rows;
    for(auto contour : contours)
    {
        double area = contourArea(contour);
        if(area < 0.001 * map_area)
        {
            continue;
        }

        for(auto q : contour)
        {
            int x = q.x;
            int y = q.y;
            x>max_x ? max_x=x : max_x=max_x;
            y>max_y ? max_y=y : max_y=max_y;
            x<min_x ? min_x=x : min_x=min_x;
            y<min_y ? min_y=y : min_y=min_y;
        }
    }

    cv::Point2i center;
    if(min_x==inputImage_gray.cols && max_x==0 && min_y==inputImage_gray.rows && max_y==0)
    {
        center = cv::Point2i(-1000,-1000);
    }
    else
    {
        cv::line(frame, cv::Point2i(min_x, min_y), cv::Point2i(max_x, min_y), color, linewidth, 8, 0);
        cv::line(frame, cv::Point2i(max_x, min_y), cv::Point2i(max_x, max_y), color, linewidth, 8, 0);
        cv::line(frame, cv::Point2i(max_x, max_y), cv::Point2i(min_x, max_y), color, linewidth, 8, 0);
        cv::line(frame, cv::Point2i(min_x, max_y), cv::Point2i(min_x, min_y), color, linewidth, 8, 0);

        center = cv::Point2i((min_x+max_x)*0.5, (min_y+max_y)*0.5);
        cv::circle(frame, center, linewidth, color, -1, 8, 0 );
        
        cv::imwrite("../files/calculate_center.jpg",frame);

    }
    
    return center;
    
}

int main()
{
    // cv::Mat image = cv::imread("../files/780.jpg", 1);

    // // The positions is in meters and are obtained with the homography matrix H.
    // cv::Mat position = (cv::Mat_ <double>(3, 1) << 
    //                         8.4568443e+00, 
    //                         3.5880664e+00,
    //                         1
    //                         );

    // cv::Mat H = (cv::Mat_ <double>(3, 3) << 
    //                 2.8128700e-02, 2.0091900e-03, -4.6693600e+00,
    //                 8.0625700e-04, 2.5195500e-02, -5.0608800e+00,
    //                 3.4555400e-04, 9.2512200e-05, 4.6255300e-01
    //                 );


    // cv::circle(image, position_to_pixel(H, position), 10, cv::Scalar(0,0,255), 2);
    // cv::imshow("frame", image);
    // cv::waitKey(0);

    // std::cout<<pixel_to_position(H, position_to_pixel(H, position))<<std::endl;

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

        cv::Point2i pixel_red = calculate_center(distinguish_color(inputImage, RED));
        std::cout<<"red: "<<pixel_red<<std::endl;
        cv::circle(inputImage, pixel_red, linewidth, cv::Scalar(0,0,255), -1, 8, 0);
        cv::circle(inputImage, pixel_red, linewidth, cv::Scalar(0,0,0), 3, 8, 0);

        cv::Point2i pixel_green = calculate_center(distinguish_color(inputImage, GREEN));
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