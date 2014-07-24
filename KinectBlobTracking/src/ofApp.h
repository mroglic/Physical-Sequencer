#pragma once

#include "Plankton.h"
#include "Ball.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCv.h" 
#include "ofxOsc.h"

// Sending OSC messages or not
#define SEND_OSC false

// The IP and port of the computer playing sound
#define HOST "192.168.101.220"
#define PORT 12345

// The number of balls we want in the simulation
#define NUM_BALLS 10
#define NUM_PLANKTONS 30

// The amount of world units the virtual camera moves by when pressing the 'w', 'a', 's', 'd' keys
#define VIRTUAL_CAMERA_TRANSLATION_STEP 100

// Whether to use a mouse controlled camera (easyCam) or a fixed camera
//#define USE_EASY_CAM 1

// Clipping planes for the real camera
#define NEAR_CLIP_REAL_CAMERA 500
#define FAR_CLIP_REAL_CAMERA 10

#define KINECT_ON true

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
	
    ofCamera virtualCam;
    bool isVirtualCamInitialAngleSet;

	ofxCv::ContourFinder contourFinder;
	bool showLabels; 

	void createBalls();
	void drawBalls();
	void updateBalls();
	std::vector<Ball> balls;

	ofFbo fboSmall, fboFinal;
    
	ofVboMesh mesh;
    int step;

	ofxOscSender sender;
	void sendBlobsPositions();
	void sendCollision(int type);

	void createNewPlankton();
	void createPlanktons();
	void updatePlanktons();
	void drawPlanktons();

	std::vector<Plankton> planktons;

	void checkCollisionAndUpdate();
	void checkCollisionAndUpdateWithBalls();
	
	bool isColliding(ofPoint c1, float r1, ofPoint c2, float r2);
};
