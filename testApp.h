#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxOpenNI.h"
#include "ofxPiavca.h"
#include "Piavca.h"
#include "ParticleSystem.h"

#define kNumTestNodes 11

using namespace xn;

class testApp : public ofBaseApp{

public:
  
	void setup();
	void update();
	void draw();
	float over(float i, float j, int x);
	float half(float i, float j);
	//float threeQuarters(float i, float j);
	void flash();
	
	ofImage stickman;
	ofxOpenNIContext context;
	ofxImageGenerator image;
	ofxDepthGenerator depth;
	ofxUserGenerator user;
	
	ofTrueTypeFont  franklinBook;
	time_t 	currentTime, startTime, flashStartTime;
	std::string	state;
	int         count, flashCount, userNum;
	bool		flashBool, check;
	ofLight		light;
	ofCamera	cam; 
		
	float timeStep, distance;
	int lineOpacity, pointOpacity;
	float particleNeighborhood, particleRepulsion;
	float centerAttraction;
	int kParticles;
	ParticleSystem particleSystem;
	bool slowMotion;
	
	XnSkeletonJointPosition jointPosHead, jointPosNeck, jointPosTorso, jointPosWaist, jointPosLCollar, jointPosLShoulder, jointPosLElbow, jointPosLWrist, jointPosLHand, jointPosLFinger, jointPosRCollar, jointPosRShoulder, jointPosRElbow, jointPosRWrist, jointPosRHand, jointPosRFinger, jointPosLHip, jointPosLKnee, jointPosLAnkle, jointPosLFoot, jointPosRHip, jointPosRKnee, jointPosRAnkle, jointPosRFoot;
	

};

#endif
