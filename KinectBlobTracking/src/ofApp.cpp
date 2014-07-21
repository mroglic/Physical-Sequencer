#include "ofApp.h"
using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init(); 
	kinect.open();		// opens first available kinect 

	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	} 

	// setup blob tracking
	contourFinder.setMinAreaRadius(1);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);	
	showLabels = true; 
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height); 

	kinect.setDepthClipping(500,10000);///nearThreshold, farThreshold);
	
	ofSetFrameRate(60); 
	
	// start from the front
	bDrawPointCloud = false;
	
	// add balls
	for (int i=0; i < NUM_BALLS; i++){
		Ball b;	
		b.setup();
		balls.push_back(b);
	}

    easyCam.setGlobalPosition(555, 4822.5, 94.5);
    
	ofVec3f orientation(-55.8, -4.1, -1.2);
	ofQuaternion qOrientation = ofQuaternion(orientation);
    easyCam.setGlobalOrientation(orientation);
    
	// open an outgoing connection to HOST:PORT
	sender.setup(HOST, PORT); 

}

//--------------------------------------------------------------


void ofApp::sendBallPosition(Ball b){
	ofxOscMessage m;
	m.setAddress("/position");	 
	m.addFloatArg(b.x);
	m.addFloatArg(b.y);
	sender.sendMessage(m);
}

void ofApp::update() {
	// send osc meesage
	for (int i=0; i < balls.size(); i++){
		balls[i].update();
		sendBallPosition(balls[i]);
	}
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		ofPixels pixels;
        
        fbo.readToPixels(pixels);
        
        grayImage.setFromPixels(pixels);

		contourFinder.findContours(grayImage);
	}  
}

void ofApp::drawBlobs() { 

	ofSetBackgroundAuto(showLabels);
	RectTracker& tracker = contourFinder.getTracker();
	CvMemStorage* storage=cvCreateMemStorage(0);

	//--v vector<vector<ofPoint>> 
//-v	CvSeq* contours;
//-v	cvFindContours(grayImage.getCvImage(),storage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0));
	/* v if (contours)
	{
		contours = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 20, 1 );//
	} */
	std::vector<ofPolyline> poliLines = contourFinder.getPolylines();  

	ofPolyline pl;
	for (int i=0; i<poliLines.size(); i++){
		pl = poliLines[i];

		pl.simplify(50);
		 
		ofPath pathFromContour;//path to be built
        for(int j = 0; j < pl.getVertices().size(); j+=1) {
            if(j == 0) {
                pathFromContour.newSubPath();
                pathFromContour.moveTo(pl.getVertices()[j]);
            } else {
                pathFromContour.lineTo(pl.getVertices()[j]);
            }
        }
        pathFromContour.close();
        pathFromContour.simplify();
        //WHY ARE THEY FILLING SO CHAOTICALLY?
        ofColor pathColor(255,255,0);
        pathFromContour.setFillColor(pathColor);
        pathFromContour.draw();


		ofFill();
		ofSetColor(255,255,0);
		pl.draw(); 
	}  

	if(showLabels) {
		ofSetColor(255);
		//kinect.draw(0, 0);		
		contourFinder.draw();		 
		for(int i = 0; i < contourFinder.size(); i++) {
			ofPoint center = toOf(contourFinder.getCenter(i));

			ofSetColor(255,0,0);
			ofFill();
			ofCircle(center.x, center.y, 10);

			ofPushMatrix(); 
			ofTranslate(center.x, center.y);
			int label = contourFinder.getLabel(i);
			string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
			ofDrawBitmapString(msg, 0, 0);
			ofVec2f velocity = toOf(contourFinder.getVelocity(i));
			ofScale(5, 5);
			ofLine(0, 0, velocity.x, velocity.y);
			ofPopMatrix();
		}
	} else {
		for(int i = 0; i < contourFinder.size(); i++) {
			unsigned int label = contourFinder.getLabel(i);
			// only draw a line if this is not a new label
			if(tracker.existsPrevious(label)) {
				// use the label to pick a random color
				ofSeedRandom(label << 24);
				ofSetColor(ofColor::fromHsb(ofRandom(255), 255, 255));
				// get the tracked object (cv::Rect) at current and previous position
				const cv::Rect& previous = tracker.getPrevious(label);
				const cv::Rect& current = tracker.getCurrent(label);
				// get the centers of the rectangles
				ofVec2f previousPosition(previous.x + previous.width / 2, previous.y + previous.height / 2);
				ofVec2f currentPosition(current.x + current.width / 2, current.y + current.height / 2);
				ofLine(previousPosition, currentPosition);
			}
		}
	}
	
	// this chunk of code visualizes the creation and destruction of labels
	const vector<unsigned int>& currentLabels = tracker.getCurrentLabels();
	const vector<unsigned int>& previousLabels = tracker.getPreviousLabels();
	const vector<unsigned int>& newLabels = tracker.getNewLabels();
	const vector<unsigned int>& deadLabels = tracker.getDeadLabels();
	ofSetColor(cyanPrint);
	for(int i = 0; i < currentLabels.size(); i++) {
		int j = currentLabels[i];
		ofLine(j, 0, j, 4);
	}
	ofSetColor(magentaPrint);
	for(int i = 0; i < previousLabels.size(); i++) {
		int j = previousLabels[i];
		ofLine(j, 4, j, 8);
	}
	ofSetColor(yellowPrint);
	for(int i = 0; i < newLabels.size(); i++) {
		int j = newLabels[i];
		ofLine(j, 8, j, 12);
	}
	ofSetColor(ofColor::white);
	for(int i = 0; i < deadLabels.size(); i++) {
		int j = deadLabels[i];
		ofLine(j, 12, j, 16);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	for (int i=0; i < balls.size(); i++){
		balls[i].draw();
	}

	drawBlobs();
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		ofPushMatrix();
		ofTranslate(640, 0);
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);		
		grayImage.draw(10, 320, 400, 300);
		ofPopMatrix();
		//contourFinder.draw(10, 320, 400, 300);  
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << 
		"virtual cam position: " << easyCam.getGlobalPosition() <<
        ", virtual cam orientation: " << easyCam.getGlobalOrientation() <<
        " | kinect accel: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Kinect probably not plugged in." << endl << endl;
    }  
    
	ofDrawBitmapString(reportStream.str(), 20, 652);    
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);	 
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.close();  
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    ofVec3f position, lookingDirection, upDirection;
    
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			

		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
        case 'w':
            position = easyCam.getPosition();
            lookingDirection = easyCam.getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            easyCam.setPosition(position + lookingDirection);
            break;
            
        case 's':
            position = easyCam.getPosition();
            lookingDirection = easyCam.getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            easyCam.setPosition(position - lookingDirection);
            break;
            
        case 'a':
            position = easyCam.getPosition();
            upDirection = easyCam.getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            easyCam.setPosition(position - upDirection);
            break;
            
        case 'd':
            position = easyCam.getPosition();
            upDirection = easyCam.getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            easyCam.setPosition(position + upDirection);
            break; 
        
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
            
//        case OF_KEY_LEFT:
//            worldZPosition -= 100;
//            break;
//            
//        case OF_KEY_RIGHT:
//            worldZPosition += 100;
//            break;

	}
}


//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{} 