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

#include "testApp.h"
//--------------------------------------------------------------
testApp::~testApp(){
	cvReleaseImage(&h);
	cvReleaseImage(&s);
	cvReleaseImage(&v);
}
void testApp::setup(){
	
	// init video input
	vidInput.initGrabber(CAM_WIDTH,CAM_HEIGHT);
	vidInput.setUseTexture(true);
	
	// window setup
	ofSetWindowShape(SCREEN_WIDTH, SCREEN_HEIGHT);
	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	ofBackground(0,0,0);
	
	// allocate stuff
	colorImg.allocate(CAM_WIDTH, CAM_HEIGHT);
	grayImg.allocate(CAM_WIDTH, CAM_HEIGHT);
	hsvImg.allocate(CAM_WIDTH, CAM_HEIGHT);
	h = cvCreateImage(cvSize(CAM_WIDTH, CAM_HEIGHT), IPL_DEPTH_8U, 1);
	s = cvCreateImage(cvSize(CAM_WIDTH, CAM_HEIGHT), IPL_DEPTH_8U, 1);
	v = cvCreateImage(cvSize(CAM_WIDTH, CAM_HEIGHT), IPL_DEPTH_8U, 1);
	
	storage = cvCreateMemStorage();
	num_mser_found = 0;
	
	/*
	 
	 typedef struct CvMSERParams
	 {
	 //! delta, in the code, it compares (size_{i}-size_{i-delta})/size_{i-delta}
	 int delta;
	 //! prune the area which bigger than maxArea
	 int maxArea;
	 //! prune the area which smaller than minArea
	 int minArea;
	 //! prune the area have simliar size to its children
	 float maxVariation;
	 //! trace back to cut off mser with diversity < min_diversity
	 float minDiversity;
	 
	 /////// the next few params for MSER of color image
	 
	 //! for color image, the evolution steps
	 int maxEvolution;
	 //! the area threshold to cause re-initialize
	 double areaThreshold;
	 //! ignore too small margin
	 double minMargin;
	 //! the aperture size for edge blur
	 int edgeBlurSize;
	 } CvMSERParams;
	 
	 CVAPI(CvMSERParams) cvMSERParams( int delta CV_DEFAULT(5), int min_area CV_DEFAULT(60),
	 int max_area CV_DEFAULT(14400), float max_variation CV_DEFAULT(.25f),
	 float min_diversity CV_DEFAULT(.2f), int max_evolution CV_DEFAULT(200),
	 double area_threshold CV_DEFAULT(1.01),
	 double min_margin CV_DEFAULT(.003),
	 int edge_blur_size CV_DEFAULT(5) );
	 */
	params = cvMSERParams( 5, .00001*CAM_WIDTH*CAM_HEIGHT, 
						  cvRound(.01*CAM_WIDTH*CAM_HEIGHT), .25, 
						  .2f, 200,
						  1.01, 
						  0.003,
						  5);
	
	mserSquares = new ofxCvColorImage[NUM_MSERS];
	for( int i = 0 ; i < NUM_MSERS; i++ )
	{
		mserSquares[i].allocate(MSER_DIM, MSER_DIM);
	}
	
	drawEllipses = true;
	drawColors = true;
	choosing_img = false;
	chosen_img = false;
	
}

void testApp::computeMSERfeatures()
{
	// convert to hsv for mser calculation
	cvCvtColor(colorImg.getCvImage(), hsvImg.getCvImage(), CV_RGB2HSV);
	
	// do the mser calculation
	mserTick = (double)cvGetTickCount();
	cvExtractMSER( hsvImg.getCvImage(), NULL, &contours, storage, params );
	mserTick = cvGetTickCount() - mserTick;
	num_mser_found = contours->total;
	
	keyptsTick = (double)cvGetTickCount();
	
	// gather keypoints into a collection:
	//vector<cv::KeyPoint> keypts;
	keypts.clear();
	
	for ( int i = 0; i < contours->total; i++ )
	{
		CvContour* r = *(CvContour**)cvGetSeqElem( contours, i );
		CvBox2D box = cvFitEllipse2( r );
		
		// make sure bounds are within the image coordinates
		if(box.center.x > 0 && box.center.x + box.size.width < CAM_WIDTH &&
		   box.center.y > 0 && box.center.y + box.size.height < CAM_HEIGHT)
		{
			// add the keypoint:
			keypts.push_back( cv::KeyPoint(box.center, 
										   sqrt( pow(box.size.width,2) + pow(box.size.height, 2) ), // might need to normalize this
										   box.angle) );
			/*
			 // get MSER and resize to square aspect ratio
			 if( i < NUM_MSERS )
			 {
			 // get ROIs
			 cvSetImageROI(colorImg.getCvImage(), cvRect(box.center.x, box.center.y, box.size.width, box.size.height));
			 
			 // create destination image
			 //   cvGetSize will return the width and the height of ROI
			 IplImage *imgRect = cvCreateImage(cvGetSize(colorImg.getCvImage()),
			 colorImg.getCvImage()->depth,
			 colorImg.getCvImage()->nChannels);
			 
			 // copy subimage (rectangular mser)
			 cvCopy(colorImg.getCvImage(), imgRect, NULL);
			 cvResetImageROI(colorImg.getCvImage());
			 
			 // copy and store mser square image
			 cvResize(imgRect, mserSquares[i].getCvImage(), CV_INTER_NN);
			 cvReleaseImage(&imgRect);
			 mserSquares[i].flagImageChanged();
			 
			 
			 box.angle = (float)CV_PI/2-box.angle;
			 
			 // draw the thing on the colorImg
			 if(drawEllipses)
			 {
			 if ( r->color > 0 )
			 cvEllipseBox( colorImg.getCvImage(), box, colors[9], 2 );
			 else
			 cvEllipseBox( colorImg.getCvImage(), box, colors[2], 2 );
			 }
			 
			 }
			 */
			
		}
		
	}  // end mser contours loop
	
	keyptsTick = cvGetTickCount() - keyptsTick;
	
	//////////////////////////////////////////////////////////////
	
	
	
	
	descTick = cvGetTickCount();
	
	cvSplit(hsvImg.getCvImage(), h, s, v, 0);
	cv::Mat grayImage = v;
	//cv::Mat descriptors;
	bool useProvidedKeypoints = true;
	sift(grayImage, cv::Mat(), keypts, descriptors, useProvidedKeypoints);
	
	descTick = cvGetTickCount() - descTick;
	
	/*
	 void SiftDescriptorExtractor::compute( const Mat& image,
	 vector<KeyPoint>& keypoints,
	 Mat& descriptors) const
	 {
	 bool useProvidedKeypoints = true;
	 Mat grayImage = image;
	 if( image.type() != CV_8U ) cvtColor( image, grayImage, CV_BGR2GRAY );
	 
	 sift(grayImage, Mat(), keypoints, descriptors, useProvidedKeypoints);
	 }
	 
	 void SIFT::operator()(const Mat& img, const Mat& mask,
	 vector<KeyPoint>& keypoints,
	 Mat& descriptors,
	 bool useProvidedKeypoints) const
	 
	 */
	
}

//--------------------------------------------------------------
void testApp::update(){
	int i;
	
	vidInput.update();
	if(vidInput.isFrameNew())
	{
		// get camera img into iplimage
		colorImg.setFromPixels(vidInput.getPixels(), CAM_WIDTH, CAM_HEIGHT);
		
		//computeMSERfeatures();
		
		

		if (chosen_img) {
			grayImg = colorImg;
			//detector.matchImages(grayImg.getCvImage(), testGrayImg.getCvImage());
			detector.setImageSearch(grayImg.getCvImage());
			detector.update();
			//detection = detector.getHomography();//matchImages(grayImg.getCvImage(), testGrayImg.getCvImage());
			//detection = detector.getTrackedPosition();
		}
		 
				
	} // end if(isFrameNew())
}

//--------------------------------------------------------------
void testApp::draw(){
	ofBackground(0,0,0);
	
	ofSetColor(255, 255, 255);
	
	// camera image
	vidInput.draw(0, 0);
	
	// draw a rectanlge around the current selection
	if (choosing_img) {
		int x = mouseX;
		int y = mouseY;
		
		ofNoFill();
		ofRect(x_start < x ? x_start : x, 
			   y_start < y ? y_start : y, 
			   abs(x_start - x), 
			   abs(y_start - y));
		
	}
	
	if (chosen_img) {
		/*
		ofNoFill();
		for (vector<MyKeypoint >::iterator it = detector.keypts2.begin(); it != detector.keypts2.end(); it++) {
			ofPushMatrix();
			ofTranslate(it->x, it->y, 0);
			ofRotate(it->ori, 0, 0, 1);
			ofRect(0, 0, it->scale/5.0f, it->scale/5.0f);
			ofPopMatrix();
		}
		*/
		ofPushMatrix();
		ofTranslate(CAM_WIDTH, 0, 0);
		testImg.draw(0, 0);
		ofPopMatrix();
		/*
		for (vector<MyKeypoint >::iterator it = detector.keypts1.begin(); it != detector.keypts1.end(); it++) {
			ofPushMatrix();
			ofTranslate(it->x, it->y, 0);
			ofRotate(it->ori, 0, 0, 1);
			ofRect(0, 0, it->scale/5.0f, it->scale/5.0f);
			ofPopMatrix();
		}
		ofPopMatrix();
		
		ofFill();
		
		if (detection > 4) {
			
			ofSetColor(200, 20, 80);
			int n = detector.match_left.size();
			for( int i = 0; i < n; i++ )
			{
				ofLine(CAM_WIDTH*2 + detector.match_left[i].x, 
					   detector.match_left[i].y, 
					   detector.match_right[i].x, 
					   detector.match_right[i].y);	
			}
		}		
		*/
		
		ofSetColor(200, 20, 50);
		
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);
		
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);
		/*
		char buf[256];
		sprintf(buf, "detection: %d\nflann: %d\n", detection, detector.use_flann);
		ofDrawBitmapString(buf, 20, CAM_HEIGHT+10);
		 */
	}
	
	// mser image
	//colorImg.draw(CAM_WIDTH, 0);
	
	
	/*
	if(drawEllipses)
	{
		ellipseTick = cvGetTickCount();
		ofNoFill();
		ofPushMatrix();
		ofTranslate(CAM_WIDTH, 0, 0);
		for (vector<cv::KeyPoint>::iterator it = keypts.begin(); it != keypts.end(); it++) 
		{
			ofPushMatrix();
			ofTranslate(it->pt.x + it->size/2, it->pt.y - it->size, 0);
			ofRotate(it->angle, 0, 0, 1);
			ofRect(0, 0, it->size, it->size);
			ofPopMatrix();	
		}
		ofPopMatrix();
		ofFill();
		ellipseTick = cvGetTickCount() - ellipseTick;
	}	
	// individual msers:
	for (int i = 0; i < NUM_MSERS; i++)
	{
		mserSquares[i].draw(CAM_WIDTH*2+10 + MSER_DIM*floor(i/((CAM_HEIGHT+50)/MSER_DIM)), MSER_DIM*(i%((CAM_HEIGHT+50)/MSER_DIM)));
	}
	
	ofSetColor(10, 120, 0);
	
	char sbuf[256];
	sprintf(sbuf, "FPS: %f", ofGetFrameRate());
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 12);
	
	sprintf(sbuf, "MSER: %g ms (found: %d)\n", mserTick/((double)cvGetTickFrequency()*1000.), num_mser_found );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 24);
	
	sprintf(sbuf, "Keypoints: %g ms\n", keyptsTick/((double)cvGetTickFrequency()*1000.) );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 36);
	
	sprintf(sbuf, "Descriptors: %g ms\n", descTick/((double)cvGetTickFrequency()*1000.) );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 48);
	
	if (drawEllipses) {
		sprintf(sbuf, "Ellipses: %g ms\n", ellipseTick/((double)cvGetTickFrequency()*1000.) );
		ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 60);
	}
	 */
	
	
}


//--------------------------------------------------------------
void testApp::keyPressed  (int key){
	
	switch (key){			
		case 's':
			vidInput.videoSettings();
			break;
		case '1':
			drawEllipses=!drawEllipses;
			break;
		case '2':
			drawColors=!drawColors;
			break;
			
		case 'b':
			break;
			
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	
	// start a rectangle selection
	if(!choosing_img)
	{
		choosing_img = true;
		x_start = x;
		y_start = y;
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	
	// end the rectangle selection
	if (choosing_img) {
		choosing_img = false;
		x_end = x;
		y_end = y;
		
		if(x_start > x_end)
			std::swap(x_start, x_end);
		if(y_start > y_end)
			std::swap(y_start, y_end);
		
		int w = x_end - x_start;
		int h = y_end - y_start;
		
		cvSetImageROI(colorImg.getCvImage(), 
					  cvRect(x_start, 
							 y_start, 
							 w, h));
		
		if (testImg.bAllocated) {
			testImg.clear();
			testGrayImg.clear();
		}
		testImg.allocate(w, h);
		testGrayImg.allocate(w, h);
		testImg = colorImg;
		testGrayImg = testImg;
		cvResetImageROI(colorImg.getCvImage());

		detector.setImageTemplate(testGrayImg.getCvImage());
		chosen_img = true;
	}
	
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
	
}