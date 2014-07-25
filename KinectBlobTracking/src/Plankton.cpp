#include "Plankton.h"

void Plankton::setup(){
	r = ofRandom(8, 20);
	 
	noiseVal = ofRandom(10,100);

	if ((ofRandom(0,1) <= 0.5)) type = 1;	
	else type = 2;	
	
	if (type == 1){		
		color = ofColor(0,0,255);
	}
	else if (type == 2){
		color = ofColor(0,255,0);
	}

	startX = ofRandom(0, ofGetWidth());
	startY = ofRandom(0, ofGetHeight());

	randomFactorX = ofRandom(0.4, 2);
	randomFactorY = ofRandom(0.4, 2);
	
	toDelete = false;
	hited = false;

	movingRange = ofRandom(30, 50);
}   

void Plankton::update(){	
	x = startX + ofMap(ofNoise(noiseVal * randomFactorX), 0, 1, -movingRange, movingRange);
	y = startY + ofMap(ofNoise(noiseVal * randomFactorY), 0, 1, -movingRange, movingRange);

	noiseVal += 0.01;

	if (hited){
		 r++;
		 if (r > 60){
			 toDelete = true;
		 }
	}	
}

void Plankton::draw(){
	ofSetColor(color);
	
	if (hited) ofNoFill();
	else ofFill();

	ofCircle(x, y, r);
} 