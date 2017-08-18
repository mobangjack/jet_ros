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
#include <string.h>
#include <stdlib.h>

void help()
{
    std::cout << "Usage: circle_detector_test [method] [color] <image_file_path>(optional, default live)" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        help();
        return 0;
    }

    int color = atoi(argv[1]);
    int method = atoi(argv[2]);
    bool live = true;

    if (argc > 3) {
        live = false;
    }

    cv::VideoCapture cap;
    if (live)
        cap.open(0);
    else
        cap.open(argv[3]);
    
    if (!cap.isOpened())
    {
        std::cout << "ERROR: can not open video capture" << std::endl;
        return -1;
    }

    cv::Mat image;
    CircleDetector detector;
    bool detected = false;

    while (cv::waitKey(30) != 'q')
    {
        cap >> image;

        if (image.empty()) break;

        detected = detector.detect(image, color, method);

        if (detected)
        {
            detector.draw(image);
        }

        imshow("circle_detecton", image);
    }

    return 0;
}