/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "circle/circle_detector.h"

CircleDetector::CircleDetector()
{

}

bool CircleDetector::detect(cv::Mat &image)
{
    bool detected = false;

    // double t = cv::getTickCount();

    cv::Mat bgr_image = image.clone();

    cv::medianBlur(bgr_image, bgr_image, 3);

    // Convert input image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(156, 50, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);

    // Combine the above two red hue images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    // Threshold the HSV image, keep only the blue pixels
    cv::Mat blue_hue_image;
    cv::inRange(hsv_image, cv::Scalar(100, 50, 50), cv::Scalar(124, 255, 255), blue_hue_image);

    // combine red hue image and blue hue image
    cv::Mat mix_hue_image;
    cv::addWeighted(red_hue_image, 1.0, blue_hue_image, 1.0, 0.0, mix_hue_image);
    
    // dialate
    cv::Mat dilate_image;
    cv::dilate(mix_hue_image, dilate_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)), cv::Point(-1, -1), 1);

    // gaussian blur
    cv::Mat blur_image;
    cv::GaussianBlur(mix_hue_image, blur_image, cv::Size(9, 9), 2, 2);

    // Use the Hough transform to detect circles in the combined threshold image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blur_image, circles, CV_HOUGH_GRADIENT, 1, blur_image.rows/8, 100, 60, 0, 0);

    //std::cout << "number of circles: " << circles.size() << std::endl;
    // Loop over all detected circles and outline them on the original image
    /*
    for(size_t i = 0; i < circles.size(); i++) {
        cv::Point center(std::round(circles[i][0]), std::round(circles[i][1]));
        int radius = std::round(circles[i][2]);
        cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
    }
    */
    
    // std::cout << "blur image channels: " << blur_image.channels() << std::endl;
    
    if (circles.size() > 0)
    {
        // Find the inner circle
        cv::Vec3f circle = circles[0];
        
        // std::cout << circle[0] << ", " << circle[1] << ", " << circle[2] << std::endl;
        
        float radius = circle[2] - 1; // inner
        int count = 0; // count > 0 : inner; count < 0 : outer
        for (float theta = 0; theta < 6.28; theta += 0.2)
        {
            
            int col = circle[0] + radius * cos(theta);
            int row = circle[1] - radius * sin(theta);
            if (col < blur_image.cols && row < blur_image.rows)
            {
                if (blur_image.at<uchar>(row, col) == 0)
                {
                    count++;
                }
                else if (blur_image.at<uchar>(row, col) == 255)
                {
                    count--;
                }
            }
        }

        m_center.x = circle[0];
        m_center.y = circle[1];
        m_radius = circle[2];
        m_inner_score = count;
        detected = true;
    }
    
    // std::cout << "count: " << count;
    
    /*
    if (count > 0)
    {
        std::cout << "count: " << count << ", inner" << std::endl;
    }
    else if (count < 0)
    {
        std::cout << "count: " << count << ", outer" << std::endl;
    }
    */

    /*    
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(blur_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    int largest_area = 0;
    int largest_contour_index = 0;
    for (int i = 0; i < contours.size(); i++)
    {
      double area = contourArea(contours[i]);
      if (area > largest_area)
      {
        largest_area = area;
        largest_contour_index = i;
      }
    }
    
    if (contours.size() > 0)
    {
      cv::Point2f center;
      float radius;
      cv::minEnclosingCircle(contours[largest_contour_index], center, radius);
      cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
    }
    */

    //t = (cv::getTickCount() - t) / cv::getTickFrequency();
    
    //std::cout << "fps: " << (1/t) << std::endl;
    
    // Show images
    /*
    cv::namedWindow("HSV image", cv::WINDOW_AUTOSIZE);
    cv::imshow("HSV image", hsv_image);
    cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Threshold lower image", lower_red_hue_range);
    cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Threshold upper image", upper_red_hue_range);
    cv::namedWindow("Threshold blue hue image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Threshold blue hue image", blue_hue_image);
    cv::namedWindow("Combined red hue threshold image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Combined red hue threshold image", red_hue_image);
    cv::namedWindow("Combined mix hue threshold image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Combined mix hue threshold image", mix_hue_image);
    cv::namedWindow("Dilate image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Dilate image", dilate_image);
    cv::namedWindow("Blur image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Blur image", blur_image);
    cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Detected red circles on the input image", orig_image);

    cv::waitKey(30);
    */

    return detected;
}

void CircleDetector::draw(cv::Mat& image)
{
    cv::circle(image, m_center, m_radius, cv::Scalar(0, 255, 0), 5);
}

