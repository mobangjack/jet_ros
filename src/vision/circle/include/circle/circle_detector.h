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

#pragma once

#include <opencv2/opencv.hpp>

class CircleDetector {
public:
    CircleDetector();
    bool detect(cv::Mat &image);
    void draw(cv::Mat& image);
protected:
    void preprocess(cv::Mat &bgr_image, cv::Mat &opt_image);
    void circleRANSAC(cv::Mat &opt_image, std::vector<cv::Vec3f> &circles, double circle_threshold, int numIterations);
    int score(cv::Mat &opt_image, cv::Vec3f& circle, int dilate, int samples);
public:
    cv::Point2f m_center;
    float m_radius;
    int m_inner_score;
};
