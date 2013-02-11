#include "testApp.h"
#include <time.h>
#include <string>
#include "ofxPiavca.h"

Piavca::Avatar *av;

/*************************************************************/
/*************************************************************/
/*************************************************************/
/*************************** SETUP ***************************/
/*************************************************************/
/*************************************************************/
/*************************************************************/

void testApp::setup(){
  
	/*************************************************************/
	/********************* set up app & screen *******************/
	/*************************************************************/
	
	ofSetFrameRate(30);
	ofBackground(30, 0, 40);
	cam.setFov(57.8);
	cam.setFarClip(10000.f);
	distance = 1500;
	
	/*************************************************************/
	/********************* load calibrate pic ********************/
	/*************************************************************/
	
	stickman.loadImage("stickmany.jpg");
	stickman.resize(stickman.width*2.2, stickman.height*2.3);
	franklinBook.loadFont("frabk.ttf", 32);
	
	/**************************************************************/
	/**************** initialise particle system ******************/
	/**************************************************************/
	
	int binPower = 2;
	particleSystem.setup((ofGetWidth()*2)/3, (ofGetHeight()*2)/3, binPower);
	kParticles = 10;
	float padding = 1;
	float maxVelocity = .5;
	
	for(int i = 0; i < kParticles * 1024; i++) {
		float x = ofRandom(padding, ofGetWidth() - padding);
		float y = ofRandom(padding, ofGetHeight() - padding);
		float xv = ofRandom(-maxVelocity, maxVelocity);
		float yv = ofRandom(-maxVelocity, maxVelocity);
		Particle particle(x, y, xv, yv);
		particleSystem.add(particle);
	}
	
	timeStep = 1;
	lineOpacity = 255;
	pointOpacity = 128;
	slowMotion = false;
	particleNeighborhood = 8;
	particleRepulsion = 2;
	centerAttraction = .005;
	
	/*************************************************************/
	/********************* set up openni kinect ******************/
	/*************************************************************/
	
	context.setupUsingXMLFile();
	image.setup(&context);
	depth.setup(&context);
	depth.getXnDepthGenerator().GetMirrorCap().SetMirror(false);
	depth.getXnDepthGenerator().GetAlternativeViewPointCap().SetViewPoint(image.getXnImageGenerator());
	user.setup(&context, &depth);
	userNum = 0;

	/*************************************************************/
	/********************* set up avatar *************************/
	/*************************************************************/
	
	ofxPiavca::loadFile("Nicola/sixteenth.xml");
	av = Piavca::Core::getCore()->getAvatar("nicola");
	check=false;
	
	/*************************************************************/
	/********************* set up counters & state ***************/
	/*************************************************************/
	
	currentTime, startTime, flashStartTime, count = 0;
	state="TRACK_FLASH";

}

/*************************************************************/
/*************************************************************/
/*************************************************************/
/*************************** UPDATE **************************/
/*************************************************************/
/*************************************************************/
/*************************************************************/

void testApp::update(){
	
	/*************************************************************/
	/*********************** update states ***********************/
	/*************************************************************/
	
	if( !user.getTrackedUsers().size() > 0 ) // no users
	{
		state="TRACK_FLASH";
		userNum=0;
		
	}
	
	if ( state=="TRACK_FLASH" ) {
		printf("user tracked: %i", userNum);
		if( user.getTrackedUsers().size()>userNum ) { 
			state="TRACK_USER";
			if (count==0&&state=="TRACK_USER") {
				time (&startTime);
				count=1;
			} 
		} else {
			// hmmmmm 
		}
		
	}
	
	
	if(state=="TRACK_USER"){			
		time (&currentTime);
		if ((currentTime>(startTime+5))) {
			printf("activated");
			state="ACTIVATE";
			count=0;
		}
		
	}
	
	if (state=="ACTIVATE") {
		timeStep = timeStep - .005;		 
		if (timeStep < .01) { 
			//timeStep = 1; 
			state="ACTIVATED";
		}
	}
	
	
	if (state=="ACTIVATED"){
		printf("about to play motion \n");
		ofxPiavca::playMotion("nicola", "transitions");
		state = "PLAYING_MOTION";
	}
	if (state=="PLAYING_MOTION") {
		printf("piavca meow \n");
		if (timeStep<1) {
			timeStep=timeStep+.005;
		}
		
		if(av->getMotionEndTime()<Piavca::Core::getCore()->getTime()){
			printf("motion ended");
			userNum=userNum+1;
			state="TRACK_FLASH";
		}
	}

	/*************************************************************/
	/********************* update kinect *************************/
	/*************************************************************/
	
	user.update();		
	context.update();
	XnUserID userId = user.getTrackedUser(userNum)->id;
	
	xn::SkeletonCapability pUserSkel = user.getXnUserGenerator().GetSkeletonCap();		
	

	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_HEAD, jointPosHead);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_NECK, jointPosNeck);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_TORSO, jointPosTorso);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_WAIST, jointPosWaist);
	
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_COLLAR, jointPosLCollar);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_SHOULDER, jointPosLShoulder);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_ELBOW, jointPosLElbow);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_WRIST, jointPosLWrist);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_HAND, jointPosLHand);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_FINGERTIP, jointPosLFinger);

	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_COLLAR, jointPosRCollar);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_SHOULDER, jointPosRShoulder);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_ELBOW, jointPosRElbow);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_WRIST, jointPosRWrist);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_HAND, jointPosRHand);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_FINGERTIP, jointPosRFinger);
	
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_HIP, jointPosLHip);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_KNEE, jointPosLKnee);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_ANKLE, jointPosLAnkle);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_LEFT_FOOT, jointPosLFoot);
	
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_HIP, jointPosRHip);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_KNEE, jointPosRKnee);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_ANKLE, jointPosRAnkle);
	pUserSkel.GetSkeletonJointPosition(userId, XN_SKEL_RIGHT_FOOT, jointPosRFoot);
	
		
}

/*************************************************************/
/*************************************************************/
/*************************************************************/
/*************************** DRAW ****************************/
/*************************************************************/
/*************************************************************/
/*************************************************************/

void testApp::draw(){
	
	/*************************************************************/
	/********************* setup scene rendering *****************/
	/*************************************************************/

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	ofEnableSmoothing();
	ofEnableAlphaBlending();
	cam.orbit(180, 0, distance, ofVec3f(0,0,distance));	
	cam.begin();
	
	/*************************************************************/
	/*********************** draw calibrate pose *****************/
	/*************************************************************/
	
	if (state=="TRACK_FLASH") 
	{
		flash();
	}
	
	/**************************************************************/
	/********** setup drawing particle system *********************/
	/**************************************************************/
	
	particleSystem.setTimeStep(timeStep);
	ofSetColor(255, 255, 255, lineOpacity);
	particleSystem.setupForces();
	glPushMatrix();
	glTranslatef(-ofGetWidth()/2, -ofGetHeight()/2, 600);
	
	// apply per-particle forces
	glBegin(GL_LINES);
	for(int i = 0; i < particleSystem.size(); i++) 
	{
		Particle& cur = particleSystem[i];
		
		// global force on other particles
		particleSystem.addRepulsionForce(cur, particleNeighborhood, particleRepulsion);
		
		// forces on this particle
		cur.bounceOffWalls(ofGetWidth()/3, 0, (ofGetWidth()*2)/3, ofGetHeight());
		cur.addDampingForce();
		
	}
	glEnd();
	
	/**************************************************************/
	/********************** draw silhouette ***********************/
	/**************************************************************/

	// kinect skeleton
	if (state=="TRACK_USER" || state=="ACTIVATE") {
		printf("kinect skeleton");
		particleSystem.addRepulsionForce((jointPosTorso.position.X+ofGetWidth())/2 , (jointPosTorso.position.Y/2.5)+(ofGetHeight()/2)+90, 90, 10);
		particleSystem.addRepulsionForce((jointPosHead.position.X+ofGetWidth())/2 , (jointPosHead.position.Y/2.5)+(ofGetHeight()/2)+90, 70, 10);		
		particleSystem.addRepulsionForce((jointPosNeck.position.X+ofGetWidth())/2 , (jointPosNeck.position.Y/2.5)+(ofGetHeight()/2)+90, 60, 10);		
		particleSystem.addRepulsionForce((half(jointPosTorso.position.X, jointPosNeck.position.X)+ofGetWidth())/2, (half(jointPosTorso.position.Y, jointPosNeck.position.Y)/2.5)+(ofGetHeight()/2)+90, 90, 10);
		particleSystem.addRepulsionForce((jointPosRShoulder.position.X+ofGetWidth())/2 , (jointPosRShoulder.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((jointPosRElbow.position.X+ofGetWidth())/2 , (jointPosRElbow.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++) 
			particleSystem.addRepulsionForce((over(jointPosRElbow.position.X,jointPosRShoulder.position.X, i)+ofGetWidth())/2, (over(jointPosRElbow.position.Y, jointPosRShoulder.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((jointPosLShoulder.position.X+ofGetWidth())/2 , (jointPosLShoulder.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((jointPosLElbow.position.X+ofGetWidth())/2 , (jointPosLElbow.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++) 
			particleSystem.addRepulsionForce((over(jointPosLElbow.position.X,jointPosLShoulder.position.X, i)+ofGetWidth())/2, (over(jointPosLElbow.position.Y, jointPosLShoulder.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((jointPosLHand.position.X+ofGetWidth())/2 , (jointPosLHand.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++) 
			particleSystem.addRepulsionForce((over(jointPosLElbow.position.X,jointPosLHand.position.X, i)+ofGetWidth())/2, (over(jointPosLElbow.position.Y, jointPosLHand.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosLFinger.position.X/2+ofGetWidth()/2 , (jointPosLFinger.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((jointPosRHand.position.X+ofGetWidth())/2 , (jointPosRHand.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++)		
			particleSystem.addRepulsionForce((over(jointPosRElbow.position.X, jointPosRHand.position.X, i)+ofGetWidth())/2, (over(jointPosRElbow.position.Y, jointPosRHand.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosRFinger.position.X/2+ofGetWidth()/2 , (jointPosRFinger.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce((half(jointPosLHip.position.X, jointPosRHip.position.X)+ofGetWidth())/2, (half(jointPosLHip.position.Y, jointPosRHip.position.Y)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosLHip.position.X/2+ofGetWidth()/2 , (jointPosLHip.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosLKnee.position.X/2+ofGetWidth()/2 , (jointPosLKnee.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++)
			particleSystem.addRepulsionForce((over(jointPosLHip.position.X,jointPosLKnee.position.X, i)+ofGetWidth())/2, (over(jointPosLHip.position.Y, jointPosLKnee.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosRHip.position.X/2+ofGetWidth()/2 , (jointPosRHip.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosRKnee.position.X/2+ofGetWidth()/2 , (jointPosRKnee.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++)
			particleSystem.addRepulsionForce((over(jointPosRHip.position.X,jointPosRKnee.position.X, i)+ofGetWidth())/2, (over(jointPosRHip.position.Y, jointPosRKnee.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosLAnkle.position.X/2+ofGetWidth()/2, (jointPosLAnkle.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++)
			particleSystem.addRepulsionForce((over(jointPosLAnkle.position.X, jointPosRKnee.position.X, i)+ofGetWidth())/2, (over(jointPosLAnkle.position.Y, jointPosRKnee.position.Y, i)/2.5)+(ofGetHeight()/2)+90, 50, 10);	
		particleSystem.addRepulsionForce(jointPosLFoot.position.X/2+ofGetWidth()/2 , (jointPosLFoot.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosRAnkle.position.X/2+ofGetWidth()/2, (jointPosRAnkle.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);
		for (int i=2; i<10; i++)
			particleSystem.addRepulsionForce((over(jointPosRAnkle.position.X,jointPosRKnee.position.X,i)+ofGetWidth())/2, (over(jointPosRAnkle.position.Y, jointPosRKnee.position.Y,i)/2.5)+(ofGetHeight()/2)+90, 50, 10);
		particleSystem.addRepulsionForce(jointPosRFoot.position.X/2+ofGetWidth()/2 , (jointPosRFoot.position.Y/2.5)+(ofGetHeight()/2)+90, 50, 10);

	}
	
	
	// piavca skeleton
	if (state=="PLAYING_MOTION") {
		printf("piavca skeleton \n");
		
		glPushMatrix();
		glTranslatef(0, 1000, 0);
		ofxPiavca::draw();
		glPopMatrix();
		
		printf("piavca meow \n");
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 Head")) ) 
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Head"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Head"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 70, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 Neck")) ) 
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Neck"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Neck"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 60, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 Pelvis")) ) 	
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Pelvis"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Pelvis"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 90, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 Spine")) ) 
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Spine"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 Spine"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 90, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Forearm")) ) {
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Forearm"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Forearm"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
			check=true;
		}
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Clavicle")) ) {
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Clavicle"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Clavicle"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		}
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Forearm")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Forearm"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Forearm"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Clavicle")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Clavicle"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Clavicle"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Upperarm")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Upperarm"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Upperarm"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Upperarm")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Upperarm"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Upperarm"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Hand")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Hand"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Hand"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Hand")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Hand"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Hand"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Thigh")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Thigh"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Thigh"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Thigh")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Thigh"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Thigh"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Calf")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Calf"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Calf"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Calf")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Calf"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Calf"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 R Foot")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Foot"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 R Foot"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
		
		if ( !av->isNull(Piavca::Core::getCore()->getJointId("Bip001 L Foot")) )
			particleSystem.addRepulsionForce((-av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Foot"), Piavca::WORLD_COORD)[0]*9)+ofGetWidth()/2, (av->getJointBasePosition(Piavca::Core::getCore()->getJointId("Bip001 L Foot"), Piavca::WORLD_COORD)[2]*9)+ofGetHeight()/2+40, 50, 10);
	
		printf("piavca meow \n");
	
	}
	
	
	particleSystem.addAttractionForce(ofGetWidth()/2, ofGetHeight()/2, ofGetWidth(), centerAttraction);
	particleSystem.update();
	ofSetColor(255, 255, 255, pointOpacity);
	particleSystem.draw();
	ofDisableAlphaBlending();
	
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(ofToString(kParticles) + "k particles", 32, 32);
	ofDrawBitmapString(ofToString((int) ofGetFrameRate()) + " fps", 32, 52);	
	glPopMatrix();

	cam.end();

	ofDisableLighting();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);	


}

void testApp::flash()
{
	if (flashCount>1000) { // stops it crashing
		flashCount=1;
	}
	flashCount++;
	printf("flashCount %i", flashCount);
		
	if ((flashCount%50)==0) {
		if (flashBool==true){
			flashBool=false;
		} else {
			flashBool=true;
			}
	}
	
	if (flashBool) {
		printf("in \n");
		glPushMatrix();
		glTranslatef(0,0,550);
		glRotatef(180, 0, 1, 0);
		glRotatef(180, 0, 0, 1);
		stickman.draw(-150, -ofGetHeight()/2+122);
		glPopMatrix();
		
			
	} else {
		glPushMatrix();
		glTranslatef(100,0,500);
		glRotatef(180, 0, 0, 1);
		ofSetHexColor(0x800080);
		franklinBook.drawString("CALIBRATE \n POSE", 0, 0);
		glPopMatrix();
		printf("out \n");
	}

}

float testApp::over(float i, float j, int x)
{
	if (i<j) 
		return i+((x*(j-i))/10); 
	else 
		return j+((x*(i-j))/10); 

}

float testApp::half(float i, float j)
{
	if (i<j) {
		return i+((j-i)/2); 
	} else {
		return j+((i-j)/2); 
	}
	
}

/*float testApp::threeQuarters(float i, float j)
{
	if (i<j) {
		return i+((3*(j-i))/4); 
	} else {
		return j+((3*(i-j))/4); 
	}
}*/
