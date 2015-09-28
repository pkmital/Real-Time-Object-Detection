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

#include "pkmImageFeatureDetector.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

pkmImageFeatureDetector::pkmImageFeatureDetector()
{
	bSetImageSearch = false;
	bSetImageTemplate = false;
	bShowImage = true;
	bTracking = false;
	bInitialized = false;
	
	winName = "correspondences";
	
	ransacReprojThreshold = 4;
	
	detector = cv::FeatureDetector::create( "DynamicSURF" ); // [“Grid”, “Pyramid”, “Dynamic”] SIFT, SURF, FAST, GFTT, MSER and HARRIS
	descriptorExtractor = cv::DescriptorExtractor::create( "SURF" ); // ["Opponent"] SIFT, SURF and BRIEF							  
	descriptorMatcher = cv::DescriptorMatcher::create( "FlannBased" ); // “FlannBased”, “BruteForceMatcher”, “BruteForce-L1” and “BruteForce-HammingLUT”
	matcherFilterType = CROSS_CHECK_FILTER;
	
}

	
void pkmImageFeatureDetector::setImageTemplate(IplImage *img_t)
{
	if(bSetImageTemplate) {	
		bSetImageTemplate = false;
		img_template_keypoints.clear();
		img_template_boundingbox.clear();
	}
	
	img_template = cv::Mat(img_t);
	
	detector->detect( img_template, img_template_keypoints );
	
	int n = img_template_keypoints.size();
	if(n > 1)
	{
		printf("[OK] Found %d keypoints.\n", n);
		bSetImageTemplate = true;
		
		
		CvPoint dst[] = { cvPoint(0, 0), 
						cvPoint(0, img_t->height), 
						cvPoint(img_t->width, img_t->height), 
						cvPoint(img_t->width, 0) };
		
		for (int i = 0; i < 4; i++) {
			img_template_boundingbox.push_back(dst[i]);
		}
	}
	else {
		printf("[ERROR] No keypoints found!\n");
		img_template_keypoints.clear();
		img_template_boundingbox.clear();
		bSetImageTemplate = false;
	}
	
	descriptorExtractor->compute( img_template, img_template_keypoints, img_template_descriptors );
	
	
}

void pkmImageFeatureDetector::setImageSearch(IplImage *img_s)
{		
	if (bSetImageSearch) {
		bSetImageSearch = false;
		img_search_keypoints.clear();
	}
	
	img_search = cv::Mat(img_s);	
	
	detector->detect( img_search, img_search_keypoints );
	
	int n = img_search_keypoints.size();
	if(n > 1)
	{
		printf("[OK] Found %d keypoints.\n", n);
		bSetImageSearch = true;
	}
	else {
		printf("[ERROR] No keypoints found!\n");
		img_search_keypoints.clear();
		bSetImageSearch = false;
	}
	
	// compute descriptors for all computed keypoints
	descriptorExtractor->compute( img_search, img_search_keypoints, img_search_descriptors );
	
}

void pkmImageFeatureDetector::simpleMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
					const cv::Mat& descriptors1, const cv::Mat& descriptors2,
					vector<cv::DMatch>& matches12 )
{
	vector<cv::DMatch> matches;
	descriptorMatcher->match( descriptors1, descriptors2, matches12 );
}

void pkmImageFeatureDetector::crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
						const cv::Mat& descriptors1, const cv::Mat& descriptors2,
						vector<cv::DMatch>& filteredMatches12, int knn )
{
	filteredMatches12.clear();
	vector<vector<cv::DMatch> > matches12, matches21;
	descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
	descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
	for( size_t m = 0; m < matches12.size(); m++ )
	{
		bool findCrossCheck = false;
		for( size_t fk = 0; fk < matches12[m].size(); fk++ )
		{
			cv::DMatch forward = matches12[m][fk];
			
			for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
			{
				cv::DMatch backward = matches21[forward.trainIdx][bk];
				if( backward.trainIdx == forward.queryIdx )
				{
					filteredMatches12.push_back(forward);
					findCrossCheck = true;
					break;
				}
			}
			if( findCrossCheck ) break;
		}
	}
}
	
void pkmImageFeatureDetector::update()
{
	if (bSetImageSearch && bSetImageTemplate) 
	{
		if (!bInitialized) 
		{
			
			
			// do matching between the template and image search
			// without tracking previous features since none initialized
			filteredMatches.clear();
			switch( matcherFilterType )
			{
				case CROSS_CHECK_FILTER :
					crossCheckMatching( descriptorMatcher, img_template_descriptors, img_search_descriptors, filteredMatches, 1 );
					break;
				default :
					simpleMatching( descriptorMatcher, img_template_descriptors, img_search_descriptors, filteredMatches );
			}
			
			// reindex based on found matches
			vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
			for( size_t i = 0; i < filteredMatches.size(); i++ )
			{
				queryIdxs[i] = filteredMatches[i].queryIdx;
				trainIdxs[i] = filteredMatches[i].trainIdx;
			}
			
			// build homograhpy w/ ransac
			vector<cv::Point2f> points1; cv::KeyPoint::convert(img_template_keypoints, points1, queryIdxs);
			vector<cv::Point2f> points2; cv::KeyPoint::convert(img_search_keypoints, points2, trainIdxs);
            if (points1.size() < 4 || points2.size() < 4) {
                printf("Not enough keypoints.\n");
                return;
            }
			H12 = findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, ransacReprojThreshold );
			
			// create a mask of the current inliers based on transform distance
			vector<char> matchesMask( filteredMatches.size(), 0 );
			
			printf("Matched %d features.\n", filteredMatches.size());
			
			// convert previous image points to current image points via homography 
			// although this is a transformation from image to world coordinates
			// it should estimate the current image points
			cv::Mat points1t; perspectiveTransform(cv::Mat(points1), points1t, H12);
			for( size_t i1 = 0; i1 < points1.size(); i1++ )
			{
				if( norm(points2[i1] - points1t.at<cv::Point2f>((int)i1,0)) < 4 ) // inlier
				{
					matchesMask[i1] = 1;
					img_search_points_inliers[1].push_back(points2[i1]);
				}
				else {
					img_search_points_outliers[1].push_back(points2[i1]);
				}
			}
			
			
			// update bounding box
			cv::Mat bb;
			perspectiveTransform(cv::Mat(img_template_boundingbox), bb, H12);
			for( int i = 0; i < 4; i++ )
			{
				dst_corners[i] = bb.at<cv::Point2f>(i,0);
				//img_template_boundingbox[i] = bb.at<cv::Point2f>(i,0);
			}
			
			/*
			// draw inliers
			drawMatches( img_search, img_template_keypoints, 
						 img_template, img_search_keypoints, 
						filteredMatches, drawImg, 
						CV_RGB(0, 255, 0), CV_RGB(0, 0, 255), matchesMask
	#if DRAW_RICH_KEYPOINTS_MODE
						, DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	#endif
						);
			
	#if DRAW_OUTLIERS_MODE
			// draw outliers
			for( size_t i1 = 0; i1 < matchesMask.size(); i1++ )
				matchesMask[i1] = !matchesMask[i1];
			drawMatches( img_template, img_template_keypoints, 
						img_search, img_search_keypoints, filteredMatches, drawImg, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0), matchesMask,
						DrawMatchesFlags::DRAW_OVER_OUTIMG | DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			
	#endif
			
			imshow( winName, drawImg );
			*/
			
		}
		else {
			// track features from previous frame into the current frame and see which 
			// features are inliers, and which are outliers.  among the features that
			// are outliers, see if any were marked as inliers in the previous frame and
			// remark then as outliers
			
			// mark decsriptors on new features marked as outliers once the number of 
			// inliers drops to a certain threshold and perform matching on the template.
			
			// patch based seems powerful as well.  creating a manifold or detecting planes 
			// in the set of inliers and tracking them as a whole may be more powerful.
			//<#statements#>
		}
		
		//std::swap(current_template_points, previous_template_points);
		std::swap(img_search_points_inliers[1], img_search_points_inliers[0]);
		std::swap(img_search_points_outliers[1], img_search_points_outliers[0]);
		swap(prev_img_search, img_search);
	} // end bSetImageSearch && bSetImageTemplate
	
}
	