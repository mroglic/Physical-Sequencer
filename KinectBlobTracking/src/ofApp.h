#pragma once

#include "Ball.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCv.h" 
#include "ofxOsc.h"

// The IP of the computer playing sound
#define HOST "192.168.101.220"
// The port of the computer playing sound
#define PORT 12345
// The number of balls we want in the simulation
#define NUM_BALLS 1
// The amount of world units the virtual camera moves by when pressing the 'w', 'a', 's', 'd' keys
#define VIRTUAL_CAMERA_TRANSLATION_STEP 100
// Whether to use a mouse controlled camera (easyCam) or a fixed camera
//#define USE_EASY_CAM 1
// Clipping planes for the real camera
#define NEAR_CLIP_REAL_CAMERA 500
#define FAR_CLIP_REAL_CAMERA 10000

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
	// Color image from the Kinect
	ofxCvColorImage colorImg;
    // Grayscale/depth image from the Kinect
	ofxCvGrayscaleImage grayImage;
	
	bool bDrawPointCloud;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    ofCamera fixedCam;
    ofCamera * activeCam;       // easyCam or fixedCam
    bool isVirtualCamInitialAngleSet;

	ofxCv::ContourFinder contourFinder;
	bool showLabels; 

	std::vector<Ball> balls;

	ofxOscSender sender;
	void sendBallPosition(Ball b); 

	ofFbo fboSmall, fboFinal;
    
	ofVboMesh mesh;
    int step;
};
