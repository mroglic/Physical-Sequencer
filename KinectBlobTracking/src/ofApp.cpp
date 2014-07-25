#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	ofSetFrameRate(60);

	if (KINECT_ON){
	
		// enable depth->video image calibration
		kinect.setRegistration(true);
		kinect.init(); 
		
		kinect.open();		// opens first available kinect 

		// setup blob tracking
		contourFinder.setMinAreaRadius(6);
		contourFinder.setMaxAreaRadius(300);
		contourFinder.setThreshold(11);

		// wait for half a frame before forgetting something
		contourFinder.getTracker().setPersistence(15);
	
		// an object can move up to 32 pixels per frame
		contourFinder.getTracker().setMaximumDistance(32);	
		showLabels = true; 
	
		colorImg.allocate(kinect.width*0.5, kinect.height*0.5);
		grayImage.allocate(kinect.width*0.5, kinect.height*0.5);

		kinect.setDepthClipping(NEAR_CLIP_REAL_CAMERA, FAR_CLIP_REAL_CAMERA);
	
		// start from the front
		bDrawPointCloud = false;

		fboSmall.allocate(320, 240, GL_RGB);
		fboFinal.allocate(640, 480, GL_RGB);
    
		step = 2;
		mesh.getVertices().resize(640/step*480/step);
		isVirtualCamInitialAngleSet = false;
		blobcolor[0].set(255,0,255);
		blobcolor[1].set(ofColor::aquamarine);
		blobcolor[2].setHex(0xFF8000, 128);
		blobcolor[3].set(0,0,255);
		blobcolor[4].set(255,0,255); 
	}

	// open an outgoing connection to HOST:PORT
	if (SEND_OSC) sender.setup(HOST, PORT); 
		
	createBalls();
	createPlanktons();
	
}

/********************************** BALLS **********************************/
void ofApp::createBalls() {	
	for (int i=0; i < NUM_BALLS; i++){
		Ball b;	
		b.setup();
		balls.push_back(b);
	}  
}

void ofApp::updateBalls() {
	for (int i = 0; i < balls.size(); i++){
 		balls[i].update();
 	} 
} 

void ofApp::drawBalls() {
	for (int i=0; i < balls.size(); i++){
 		balls[i].draw();
 	}
}

/********************************** PLANKTONS **********************************/
void ofApp::createPlanktons(){	
	for(int i = 0; i < NUM_PLANKTONS; i++) {
		createNewPlankton();
	}
}

void ofApp::createNewPlankton(){
	Plankton pl;
	pl.setup();
	planktons.push_back(pl);
}

void ofApp::updatePlanktons(){
	for(int i = 0; i < planktons.size(); i++) {
		//Plankton p = planktons[i];
		// p.update();
		planktons[i].update();
	}
}

void ofApp::drawPlanktons(){
	for(int i = 0; i < planktons.size(); i++) {	
		Plankton p = planktons[i];
		p.draw();
	}
}

/********************************** OSC **********************************/
void ofApp::sendBlobsPositions(){
	if (SEND_OSC){
		for(int i = 0; i < blobs.size(); i++) {
			ofxOscMessage m;
			m.setAddress("/blobs/" + ofToString(i));
			// send id, x and y of blobs
			m.addIntArg(blobs[i].id);
			m.addFloatArg(blobs[i].x);
			m.addFloatArg(blobs[i].y);
			m.addFloatArg(blobs[i].r); 
			sender.sendMessage(m);
		}  
	}
} 

void ofApp::sendEntered(int label){
	if (SEND_OSC){
		ofxOscMessage m;
		m.setAddress("/entered");		
		m.addIntArg(label);		
		sender.sendMessage(m);
	}
}

void ofApp::sendExited(int label){
	if (SEND_OSC){
		ofxOscMessage m;
		m.setAddress("/exited");		
		m.addIntArg(label);		 
		sender.sendMessage(m);
	}
}

void ofApp::sendCollision(int type){
	if (SEND_OSC){
		ofxOscMessage m;
		m.setAddress("/collision");
		// send type of collision
		m.addIntArg(type);		 
		sender.sendMessage(m);
	}
} 

/********************************** UPDATE **********************************/
void ofApp::update() {	
	if (KINECT_ON){
    
		if (!isVirtualCamInitialAngleSet) {
			if (kinect.getCurrentCameraTiltAngle() != 0) {
				virtualCam.setPosition(0, 3300, -500);
				ofVec3f orientation(-90-kinect.getCurrentCameraTiltAngle(), 0, 0);
				virtualCam.setOrientation(orientation);
				isVirtualCamInitialAngleSet = true;
			}
		}

		kinect.update();

		//  get blobs
		if (kinect.isFrameNewDepth()) findContours();  
		
		// update blobs
		updateBlobs();

		// send blob positions
		sendBlobsPositions();
	}
	else{
		updateBalls();
	}
	
	updatePlanktons();
	
	if (KINECT_ON) checkCollisionAndUpdate();
	else checkCollisionAndUpdateWithBalls();
	
} 

/********************************** COLLISION **********************************/
void ofApp::checkCollisionAndUpdate(){
	for(int i = 0; i < contourFinder.size(); i++) {
		ofPoint center = toOf(contourFinder.getCenter(i));

		for(int j = 0; j < planktons.size(); j += 1) {			
			ofPoint c1(center.x * 4, center.y * 4);
			ofPoint c2(planktons[j].x, planktons[j].y);
					
			// delete plankton after hited and animation
			// create new plankton
			if (planktons[j].hited){
				if (planktons[j].toDelete){
					 planktons.erase(planktons.begin() + j);
					 createNewPlankton();
				}
			}
			// hited if colliding
			else{
				if (isColliding(c1, 30, c2, planktons[j].r)){				 
					planktons[j].hited = true;
					// send hited osc message with type
					sendCollision(planktons[j].type);
				}
			}			
		}
	}
}

void ofApp::checkCollisionAndUpdateWithBalls(){
	for(int i = 0; i < balls.size(); i += 1) {		
		for(int j = 0; j < planktons.size(); j += 1) {			
			ofPoint c1(balls[i].x, balls[i].y);
			ofPoint c2(planktons[j].x, planktons[j].y);
					
			// delete plankton after hited and animation
			// create new plankton
			if (planktons[j].hited){
				if (planktons[j].toDelete){
					 planktons.erase(planktons.begin() + j);
					 createNewPlankton();
				}
			}
			// hited if colliding
			else{
				if (isColliding(c1,  balls[i].r, c2, planktons[j].r)){				 
					planktons[j].hited = true;
					// send hited osc message with type
					sendCollision(planktons[j].type);
				}
			}			
		}
	}
}

/********************************** BLOBS **********************************/

void ofApp::updateBlobs() {
	if (contourFinder.size() == 0){
		// make new array of blobs
		std::vector<Blob> blobs;
		return;
	} 
			
	// check new blobs
	for(int i = 0; i < contourFinder.size(); i++){
		unsigned int blobId = contourFinder.getLabel(i);
		
		ofPoint center = toOf(contourFinder.getCenter(i));
		float x = center.x * 4;
		float y = center.y * 4;
		
		float r = contourFinder.getContourArea(i) / 20;

		// is new blob
		bool isNewBlob = true;
		for(int j = 0; j < blobs.size(); j++){						
			if (blobs[j].id == blobId) {
				isNewBlob = false;				
				blobs[j].update(x, y, r);
				break;
			}
		}

		// add new blob
		if (isNewBlob){
			Blob b;
			b.setup(blobId, x, y);
			b.update(x, y, r);
			blobs.push_back(b);
			sendEntered(i);
			printf("NEW BLOB ADDED \n");
		}	
	}				

	// check ofscreen blobs
	for(int j = blobs.size()-1; j >= 0; j--){
		// is ofscreen blob
		bool isOffscreenBlob = true;
		for(int i = 0; i < contourFinder.size(); i++){
			unsigned int blobId = contourFinder.getLabel(i);
			if (blobs[j].id == blobId) {
				isOffscreenBlob = false;
				break;					
			}
		}

		// remove offscreen blob
		if (isOffscreenBlob){
			blobs.pop_back();
			sendExited(j);
			printf("REMOVED BLOB \n");
		} 
	}

	//printf("BLOB SIZE %d \n", blobs.size());
}

void ofApp::findContours() {
	// there is a new frame and we are connected	 
    if (true) {
        ofPixels pixels;
        fboSmall.readToPixels(pixels);
        colorImg.setFromPixels(pixels);
        grayImage = colorImg;
    } else {
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    }	
       
	// get blobs
	contourFinder.findContours(grayImage);

	
}

void ofApp::drawBlobs2() {	
	for (int i = 0; i < blobs.size(); i++){
		blobs[i].draw();
	}
}

void ofApp::drawBlobs() {
	ofSetBackgroundAuto(showLabels);
	RectTracker& tracker = contourFinder.getTracker();	
	
	if(showLabels) {
		ofSetColor(255);
		//kinect.draw(0, 0);		
		contourFinder.draw();
		for(int i = 0; i < contourFinder.size(); i++) {
			ofPoint center = toOf(contourFinder.getCenter(i));

			ofSetColor(255,0,0);
			ofFill();

			center.x *= 4;
			center.y *= 4;
			ofCircle(center.x, center.y, 30);

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

/********************************** DRAW **********************************/

void ofApp::draw() {
	ofBackground(0);
	ofSetColor(255, 255, 255);
    ofDisableAlphaBlending();
    
	if (KINECT_ON){
		if(true) {
			fboFinal.begin();
				ofClear(0,255);
				virtualCam.begin(ofRectangle(0, 0, 640, 480));
					drawPointCloud();
				virtualCam.end();
			fboFinal.end();

			fboSmall.begin();
				ofClear(0,255);
				fboFinal.draw(0, 0, 320, 240);
			fboSmall.end();

			fboSmall.draw(0,0);

			for(unsigned int j = 0; j < contourFinder.size(); j++){
				//contourFinder.setSimplify(true);
				rectcontour = contourFinder.getFitEllipse(j);
				cv::Point2f rc=rectcontour.center;
				cv::Size2f rs=rectcontour.size;
				float rangle=rectcontour.angle;
				ofPushMatrix();
				ofTranslate(rc.x*4,rc.y*4);
				ofRotate(rangle);
			
				for(float theta = 0; theta < TWO_PI; theta += 0.1){
					ofPath p;//,p1;
					ofColor stupidcolor(255,255,255);
					p.setColor(blobcolor[j]);
					float ww1 = ofRandom(20);
					float wigglywidth=rs.width*0.75*cos(theta);
					float wigglyheight=rs.height*0.75*sin(theta);
					p.circle(wigglywidth+ww1,wigglyheight+ww1,1);
					p.setFilled(true);
					p.setCircleResolution(100);

					ofEnableAlphaBlending();
					ofNoFill();
					ofSetColor(255,0,0,128);    
					ofEllipse(0,0,wigglywidth+ww1,wigglyheight+ww1);
					ofNoFill();
					ofDisableAlphaBlending();
					p.draw(0,0);
				}
				ofPopMatrix();	
			}
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
		
		//grayImage.draw(640, 0); 
	
		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;
        
		if(kinect.hasAccelControl()) {
			reportStream << 
			"virtual cam position: " << virtualCam.getPosition() <<
			", virtual cam orientation: " << virtualCam.getOrientationEuler() <<
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

	drawPlanktons();

	if (KINECT_ON) drawBlobs2();
	else drawBalls();
}

bool ofApp::isColliding(ofPoint c1, float r1, ofPoint c2, float r2){	
	float dx = c1.x - c2.x;
	float dy = c1.y - c2.y;
	
	float dist = r1 + r2;
	
	return (dx * dx + dy * dy <= dist * dist);
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

void ofApp::exit() {
	if (KINECT_ON) kinect.close(); 
}

void ofApp::keyPressed (int key) {
    ofVec3f position, lookingDirection, upDirection;
    
	switch (key) {
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
						
        case 'w':
            position = virtualCam.getPosition();
            lookingDirection = virtualCam.getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            virtualCam.setPosition(position + lookingDirection);
            break;
            
        case 's':
            position = virtualCam.getPosition();
            lookingDirection = virtualCam.getLookAtDir();
            lookingDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            virtualCam.setPosition(position - lookingDirection);
            break;
            
        case 'a':
            position = virtualCam.getPosition();
            upDirection = virtualCam.getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            virtualCam.setPosition(position - upDirection);
            break;
            
        case 'd':
            position = virtualCam.getPosition();
            upDirection = virtualCam.getUpDir();
            upDirection *= VIRTUAL_CAMERA_TRANSLATION_STEP;
            virtualCam.setPosition(position + upDirection);
            break;
			
		case OF_KEY_UP:
            virtualCam.rotate(1, virtualCam.getSideDir());
			break;
			
		case OF_KEY_DOWN:
            virtualCam.rotate(-1, virtualCam.getSideDir());
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