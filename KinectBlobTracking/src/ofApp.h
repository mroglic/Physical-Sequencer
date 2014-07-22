#pragma once

#include "Ball.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCv.h" 
#include "ofxOsc.h"

#define HOST "192.168.101.220"
#define PORT 12345 

#define NUM_BALLS 1
#define VIRTUAL_CAMERA_TRANSLATION_STEP 100
//#define USE_EASY_CAM 1

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    
    void findContours();
	void drawBlobs();
	
	ofxKinect kinect; 
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image 	 
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    ofCamera fixedCam;
    ofCamera * activeCam;

	// blob tracking
	float threshold;
	ofVideoPlayer movie;	
	ofxCv::ContourFinder contourFinder;
	bool showLabels; 

	std::vector<Ball> balls;

	ofxOscSender sender;
	void sendBallPosition(Ball b); 

	ofFbo fboSmall, fboFinal;
 
    bool isEasyCamInitialPositionSet;
    
	ofVboMesh mesh;
    int step;
};
