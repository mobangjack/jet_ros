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

 #include "param.h"

void Param::load(std::string file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fs["extrinsic_rotation"] >> cv_R;
    fs["extrinsic_translation"] >> cv_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    ROS_INFO_STREAM("Extrinsic_R : " << std::endl << eigen_R);
    ROS_INFO_STREAM("Extrinsic_T : " << std::endl << eigen_T.transpose());
    fs.release();
}
