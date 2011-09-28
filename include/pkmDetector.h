/*
 *  Created by Parag K. Mital - http://pkmital.com 
 *  Contact: parag@pkmital.com
 *
 *  Copyright 2011 Parag K. Mital. All rights reserved.
 * 
 *	Permission is hereby granted, free of charge, to any person
 *	obtaining a copy of this software and associated documentation
 *	files (the "Software"), to deal in the Software without
 *	restriction, including without limitation the rights to use,
 *	copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the
 *	Software is furnished to do so, subject to the following
 *	conditions:
 *	
 *	The above copyright notice and this permission notice shall be
 *	included in all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,	
 *	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *	OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <cstdio>
#include <cstring>

//#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

#define DRAW_RICH_KEYPOINTS_MODE     1
#define DRAW_OUTLIERS_MODE           1

class pkmDetector
{
	
enum { NONE_FILTER = 0, CROSS_CHECK_FILTER = 1 };
	
	public:
	
	pkmDetector();
	
	void setImageTemplate(IplImage *img_t);
	void setImageSearch(IplImage *img_s);
	
	void simpleMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
						const cv::Mat& descriptors1, const cv::Mat& descriptors2,
						vector<cv::DMatch>& matches12 );	
	void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
							const cv::Mat& descriptors1, const cv::Mat& descriptors2,
							vector<cv::DMatch>& filteredMatches12, int knn=1 );
	
	void update();
	void doIteration( const cv::Mat& img1, Mat& img2, 
					 vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
					 cv::Ptr<cv::FeatureDetector>& detector, cv::Ptr<cv::DescriptorExtractor>& descriptorExtractor,
					 cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher, int matcherFilter,
					 double ransacReprojThreshold);	
	
	
	vector<cv::Point2f>								img_search_points_inliers[2], img_search_points_outliers[2]; 
	vector<cv::Point2f> 							img_template_boundingbox;
	cv::Point2f										src_corners[4], dst_corners[4], prev_dst_corners[4];
	
private:
	
	int												matcherFilterType;
	
	bool											bSetImageTemplate, bSetImageSearch;
	
	cv::Ptr<cv::FeatureDetector>					detector; //= FeatureDetector::create( argv[1] );
    cv::Ptr<cv::DescriptorExtractor>				descriptorExtractor; //= DescriptorExtractor::create( argv[2] );
    cv::Ptr<cv::DescriptorMatcher>					descriptorMatcher; //= DescriptorMatcher::create( argv[3] );
	
	cv::Mat											img_template, img_search;
	
	bool											bInitialized, bTracking;
	cv::Mat											prev_img_search;
	
	vector<cv::KeyPoint>							img_search_keypoints, img_template_keypoints;
	vector<cv::KeyPoint>							img_search_keypoints_inliers[2], img_search_keypoints_outliers[2];
	
	cv::Mat											img_search_descriptors, img_template_descriptors;
	
	vector<cv::DMatch>								filteredMatches;
	
	float											ransacReprojThreshold;
	
	
    cv::Mat											H12;
	
	bool											bShowImage;
    cv::Mat											drawImg;
	string											winName;
	
};