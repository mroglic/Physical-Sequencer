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

	// setup blob tracking
	contourFinder.setMinAreaRadius(1);
	contourFinder.setMaxAreaRadius(1000);
	contourFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);	
	showLabels = true; 
	
	colorImg.allocate(kinect.width*0.5, kinect.height*0.5);
	grayImage.allocate(kinect.width*0.5, kinect.height*0.5);

	kinect.setDepthClipping(NEAR_CLIP_REAL_CAMERA, FAR_CLIP_REAL_CAMERA);
	
	ofSetFrameRate(60); 
	
	// start from the front
	bDrawPointCloud = false;
	
	// add balls
	for (int i=0; i < NUM_BALLS; i++){
		Ball b;	
		b.setup();
		balls.push_back(b);
	}
    
#ifdef USE_EASY_CAM
    activeCam = &easyCam;
#else
    activeCam = &fixedCam;
#endif

    easyCam.setPosition(0., 0., 0.);
    ofVec3f orientation(0., 0., 0.);
    easyCam.setOrientation(orientation);
    
	// open an outgoing connection to HOST:PORT
	sender.setup(HOST, PORT); 

    fboSmall.allocate(320, 240, GL_RGB);
    fboFinal.allocate(640, 480, GL_RGB);
    
	step = 2;
    mesh.getVertices().resize(640/step*480/step);
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
    
    if (!isVirtualCamInitialAngleSet) {
        if (kinect.getCurrentCameraTiltAngle() != 0) {
            activeCam->setPosition(0, 3300, -500);
            ofVec3f orientation(-90-kinect.getCurrentCameraTiltAngle(), 0, 0);
            activeCam->setOrientation(orientation);
            isVirtualCamInitialAngleSet = true;
        }
    }
    
	ofBackground(100, 100, 100);
	
	kinect.update();
}

void ofApp::findContours() {
	// there is a new frame and we are connected
	if(kinect.isFrameNewDepth()) {
        if (true) {
            ofPixels pixels;
            fboSmall.readToPixels(pixels);
            colorImg.setFromPixels(pixels);
            grayImage = colorImg;
        } else {
            // load grayscale depth image from the kinect source
            grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        }
        
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
//        pathFromContour.draw();


		ofFill();
		ofSetColor(255,255,0);
//		pl.draw(); 
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

	ofSetColor(255, 255, 255);
    ofDisableAlphaBlending();
    
	if(true) {
        fboFinal.begin();
            ofClear(0,255);
            activeCam->begin(ofRectangle(0, 0, 640, 480));
                drawPointCloud();
            activeCam->end();
        fboFinal.end();
        fboSmall.begin();
            ofClear(0,255);
            fboFinal.draw(0, 0, 320, 240);
        fboSmall.end();
        fboSmall.draw(0,0);
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
    
    findContours();
    grayImage.draw(640, 0);
    drawBlobs();
    ofEnableAlphaBlending();
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << 
		"virtual cam position: " << activeCam->getPosition() <<
        ", virtual cam orientation: " << activeCam->getOrientationEuler() <<
        " getAccelPitch: " << kinect.getAccelPitch() <<
        " | kinect accel: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Kinect probably not plugged in." << endl << endl;
    }  
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    ofDrawBitmapString(ofToString(ofGetFrameRate()),20,20);
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	for(int y = 0, i = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step, i++) {
            mesh.getVertices()[i] = kinect.getWorldCoordinateAt(x, y);
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
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
						
        case 'w':
            position = activeCam->getPosition();
            lookingDirection = activeCam->getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            activeCam->setPosition(position + lookingDirection);
            break;
            
        case 's':
            position = activeCam->getPosition();
            lookingDirection = activeCam->getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            activeCam->setPosition(position - lookingDirection);
            break;
            
        case 'a':
            position = activeCam->getPosition();
            upDirection = activeCam->getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            activeCam->setPosition(position - upDirection);
            break;
            
        case 'd':
            position = activeCam->getPosition();
            upDirection = activeCam->getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            activeCam->setPosition(position + upDirection);
            break;
			
		case OF_KEY_UP:
            activeCam->rotate(1, activeCam->getSideDir());
			break;
			
		case OF_KEY_DOWN:
            activeCam->rotate(-1, activeCam->getSideDir());
			break;
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