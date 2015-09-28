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
    
	choosing_img = false;
	chosen_img = false;
	
}

//--------------------------------------------------------------
void testApp::update(){
	int i;
	
	vidInput.update();
	if(vidInput.isFrameNew())
	{
		// get camera img into iplimage
		colorImg.setFromPixels(vidInput.getPixels(), CAM_WIDTH, CAM_HEIGHT);
		
		if (chosen_img) {
			grayImg = colorImg;
			detector.setImageSearch(grayImg.getCvImage());
			detector.update();
		}
		 
				
	} 
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

		ofPushMatrix();
		ofTranslate(CAM_WIDTH, 0, 0);
		testImg.draw(0, 0);
		ofPopMatrix();
		
		ofSetColor(200, 20, 50);
		
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);
		
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);

	}
	
	
}


//--------------------------------------------------------------
void testApp::keyPressed  (int key){
	
	switch (key){			
		case 's':
			vidInput.videoSettings();
			break;
		case '1':
			break;
		case '2':
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