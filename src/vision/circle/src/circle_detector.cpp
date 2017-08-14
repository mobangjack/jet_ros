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

#define SHOW_IMG 0
#define SHOW_FPS 1

CircleDetector::CircleDetector()
{

}

void CircleDetector::preprocess(cv::Mat &bgr_image, cv::Mat &opt_image)
{
    opt_image = bgr_image.clone();

    // median blur
    cv::medianBlur(opt_image, opt_image, 3);

    // Convert input image to HSV
    cv::cvtColor(opt_image, opt_image, cv::COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_image;
    cv::Mat upper_red_hue_image;
    cv::inRange(opt_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lower_red_hue_image);
    cv::inRange(opt_image, cv::Scalar(156, 50, 50), cv::Scalar(180, 255, 255), upper_red_hue_image);

    // Combine the above two red hue images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_image, 1.0, upper_red_hue_image, 1.0, 0.0, red_hue_image);

    // Threshold the HSV image, keep only the blue pixels
    cv::Mat blue_hue_image;
    cv::inRange(opt_image, cv::Scalar(100, 50, 50), cv::Scalar(124, 255, 255), blue_hue_image);

    // combine red hue image and blue hue image
    cv::addWeighted(red_hue_image, 1.0, blue_hue_image, 1.0, 0.0, opt_image);
#if (SHOW_IMG)
    imshow("lower_red_hue_image", lower_red_hue_image);
    imshow("upper_red_hue_image", upper_red_hue_image);
    imshow("red_hue_image", red_hue_image);
    imshow("blue_hue_image", blue_hue_image);
    imshow("mix_hue_image", opt_image);
    waitKey(1);
#endif   
    // dialate
    cv::dilate(opt_image, opt_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)), cv::Point(-1, -1), 1);
#if (SHOW_IMG)
    imshow("dilate_image", opt_image);
    waitKey(1);
#endif 
    // gaussian blur
    cv::GaussianBlur(opt_image, opt_image, cv::Size(9, 9), 2, 2);
#if (SHOW_IMG)
    imshow("blur_image", opt_image);
    waitKey(1);
#endif 
}

// circleRANSAC
//
// input:
//		circles - return vector of Vec3f (x,y,radius)
//		circle_threshold - value between 0 and 1 for the percentage of the circle that needs to vote for it to be accepted
//		numIterations - the number of RANSAC loops, the function will quit if there is no points left in the set
//
void CircleDetector::circleRANSAC(cv::Mat &opt_image, std::vector<cv::Vec3f> &circles, double circle_threshold, int numIterations)
{
	circles.clear();
	
	// Create point set
	std::vector<cv::Point2d> points;
	for(int r = 0; r < opt_image.rows; r++)
	{
		for(int c = 0; c < opt_image.cols; c++)
		{
			if(opt_image.at<unsigned char>(r,c) == 255)
			{
				points.push_back(cv::Point2d(c,r));
			}
		}	
	}
	
	// 4 point objects to hold the random samples
	cv::Point2d pointA;
	cv::Point2d pointB;
	cv::Point2d pointC;
	cv::Point2d pointD;
	
	// distances between points
	double AB;
	double BC;
	double CA;
	double DC;

	// varibales for line equations y = mx + b
	double m_AB;
	double b_AB;
	double m_BC;
	double b_BC;

	// varibles for line midpoints
	double XmidPoint_AB;
	double YmidPoint_AB;
	double XmidPoint_BC;
	double YmidPoint_BC;

	// variables for perpendicular bisectors
	double m2_AB;
	double m2_BC;
	double b2_AB;
	double b2_BC;

	// RANSAC
	cv::RNG rng; 
	int min_point_separation = 10; // change to be relative to image size?
	int colinear_tolerance = 1; // make sure points are not on a line
	int radius_tolerance = 3; // change to be relative to image size?
	int points_threshold = 10; //should always be greater than 4
	//double min_circle_separation = 10; //reject a circle if it is too close to a previously found circle
	//double min_radius = 10.0; //minimum radius for a circle to not be rejected
	
	int x, y;
	cv::Point2d center;
	double radius;
	
	// Iterate
	for(int iteration = 0; iteration < numIterations; iteration++) 
	{
		//std::cout << "RANSAC iteration: " << iteration << std::endl;
		
		// get 4 random points
		pointA = points[rng.uniform((int)0, (int)points.size())];
		pointB = points[rng.uniform((int)0, (int)points.size())];
		pointC = points[rng.uniform((int)0, (int)points.size())];
		pointD = points[rng.uniform((int)0, (int)points.size())];
		
		// calc lines
		AB = norm(pointA - pointB);
		BC = norm(pointB - pointC);
		CA = norm(pointC - pointA);
		DC = norm(pointD - pointC);
		
		// one or more random points are too close together
		if(AB < min_point_separation || BC < min_point_separation || CA < min_point_separation || DC < min_point_separation) continue;
		
		//find line equations for AB and BC
		//AB
		m_AB = (pointB.y - pointA.y) / (pointB.x - pointA.x + 0.000000001); //avoid divide by 0
		b_AB = pointB.y - m_AB*pointB.x;

		//BC
		m_BC = (pointC.y - pointB.y) / (pointC.x - pointB.x + 0.000000001); //avoid divide by 0
		b_BC = pointC.y - m_BC*pointC.x;
		
		
		//test colinearity (ie the points are not all on the same line)
		if(abs(pointC.y - (m_AB*pointC.x + b_AB + colinear_tolerance)) < colinear_tolerance) continue;
		
		//find perpendicular bisector
		//AB
		//midpoint
		XmidPoint_AB = (pointB.x + pointA.x) / 2.0;
		YmidPoint_AB = m_AB * XmidPoint_AB + b_AB;
		//perpendicular slope
		m2_AB = -1.0 / m_AB;
		//find b2
		b2_AB = YmidPoint_AB - m2_AB*XmidPoint_AB;

		//BC
		//midpoint
		XmidPoint_BC = (pointC.x + pointB.x) / 2.0;
		YmidPoint_BC = m_BC * XmidPoint_BC + b_BC;
		//perpendicular slope
		m2_BC = -1.0 / m_BC;
		//find b2
		b2_BC = YmidPoint_BC - m2_BC*XmidPoint_BC;
		
		//find intersection = circle center
		x = (b2_AB - b2_BC) / (m2_BC - m2_AB);
		y = m2_AB * x + b2_AB;	
		center = cv::Point2d(x,y);
		radius = cv::norm(center - pointB);
#if (SHOW_IMG)		
		/// geometry debug image
		if(true)
		{
			Mat debug_image = blur_image.clone();
			cvtColor(debug_image, debug_image, CV_GRAY2RGB);
		
			Scalar pink(255,0,255);
			Scalar blue(255,0,0);
			Scalar green(0,255,0);
			Scalar yellow(0,255,255);
			Scalar red(0,0,255);
		
			// the 3 points from which the circle is calculated in pink
			circle(debug_image, pointA, 3, pink);
			circle(debug_image, pointB, 3, pink);
			circle(debug_image, pointC, 3, pink);
		
			// the 2 lines (blue) and the perpendicular bisectors (green)
			line(debug_image,pointA,pointB,blue);
			line(debug_image,pointB,pointC,blue);
			line(debug_image,Point(XmidPoint_AB,YmidPoint_AB),center,green);
			line(debug_image,Point(XmidPoint_BC,YmidPoint_BC),center,green);
		
			circle(debug_image, center, 3, yellow); // center
			circle(debug_image, center, radius, yellow);// circle
		
			// 4th point check
			circle(debug_image, pointD, 3, red);
		
			imshow("ransac debug", debug_image);
			waitKey(1);
		}
#endif		
		//check if the 4 point is on the circle
		if(abs(cv::norm(pointD - center) - radius) > radius_tolerance) continue;
				
		// vote
		std::vector<int> votes;
		std::vector<int> no_votes;
		for(int i = 0; i < (int)points.size(); i++) 
		{
			double vote_radius = norm(points[i] - center);
			
			if(abs(vote_radius - radius) < radius_tolerance) 
			{
				votes.push_back(i);
			}
			else
			{
				no_votes.push_back(i);
			}
		}
		
		// check votes vs circle_threshold
		if( (float)votes.size() / (2.0*CV_PI*radius) >= circle_threshold )
		{
			circles.push_back(cv::Vec3f(x,y,radius));
#if (SHOW_IMG)			
			// voting debug image
			if(true)
			{
				Mat debug_image2 = blur_image.clone();
				cvtColor(debug_image2, debug_image2, CV_GRAY2RGB);
		
				Scalar yellow(0,255,255);
				Scalar green(0,255,0);
			
				circle(debug_image2, center, 3, yellow); // center
				circle(debug_image2, center, radius, yellow);// circle
			
				// draw points that voted
				for(int i = 0; i < (int)votes.size(); i++)
				{
					circle(debug_image2, points[votes[i]], 1, green);
				}
			
				imshow("ransac debug", debug_image2);
				waitKey(1);
			}
#endif			
			// remove points from the set so they can't vote on multiple circles
			std::vector<cv::Point2d> new_points;
			for(int i = 0; i < (int)no_votes.size(); i++)
			{
				new_points.push_back(points[no_votes[i]]);
			}
			points.clear();
			points = new_points;		
		}
		
		// stop RANSAC if there are few points left
		if((int)points.size() < points_threshold)
			break;
	}
}

int CircleDetector::score(cv::Mat &opt_image, cv::Vec3f& circle, int dilate, int samples)
{
    int count = 0; // count > 0 : inner; count < 0 : outer
    float step = 6.28f / samples;
    float radius = circle[2] + dilate; // dilate < 0 : inner, dilate > 0 : outer
    for (float theta = 0; theta < 6.28f; theta += step)
    {
        int col = circle[0] + radius * cos(theta);
        int row = circle[1] - radius * sin(theta);
        if (col < opt_image.cols && row < opt_image.rows)
        {
            if (opt_image.at<uchar>(row, col) == 255)
            {
                count++;
            }
            else if (opt_image.at<uchar>(row, col) == 0)
            {
                count--;
            }
        }
    }
    return count;
}

bool CircleDetector::detect(cv::Mat &image, int method = CIRCLE_DETECTION_METHOD_HOUGH)
{
    bool detected = false;
#if (SHOW_FPS)
    double t = cv::getTickCount();
#endif
    cv::Mat opt_image;
    
    preprocess(image, opt_image);

    std::vector<cv::Vec3f> circles;
    
    if (method == CIRCLE_DETECTION_METHOD_RANSAC)
    {
        // Use RANSAC to detect circles
        circleRANSAC(opt_image, circles, 0.99, 1);
    }
    else
    {
        // Use the Hough transform to detect circles
        cv::HoughCircles(opt_image, circles, CV_HOUGH_GRADIENT, 1, opt_image.rows/8, 100, 60, 0, 0);
    }
    
    if (circles.size() > 0)
    {
        // Find the inner circle
        cv::Vec3f circle = circles[0];

        m_center.x = circle[0];
        m_center.y = circle[1];
        m_radius = circle[2];
        m_inner_score = score(opt_image, circle, -1, 30);
        detected = true;
    }
#if (SHOW_FPS)    
    t = (cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "fps: " << (1/t) << std::endl;
#endif   
    return detected;
}

void CircleDetector::draw(cv::Mat& image)
{
    cv::circle(image, m_center, m_radius, cv::Scalar(0, 255, 0), 5);
}

