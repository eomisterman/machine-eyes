import SimpleOpenNI.*;
import oscP5.*;
import netP5.*;
import KinectProjectorToolkit.*;
import java.util.*;
import java.lang.Math;

// --------------------------------------------------------------------------------
//  MAIN PROGRAM
// --------------------------------------------------------------------------------

boolean kDrawSkeleton = true; // << set to true to draw skeleton, false to use Eye animation
KinectProjectorToolkit kpc;
int MRC = 1;
private static ArrayList<Eye> eyeList = new ArrayList<Eye>();
PVector com = new PVector();
PVector com2d = new PVector();
int dispHeight;
int dispWidth;
float[] data = new float[45];


void setup() {
  dispWidth = displayWidth;
  dispHeight = displayHeight;
  // Set size of window
  size(dispWidth, dispHeight, P3D);
  // Set size of canvas within window
  canvas = createGraphics(dispWidth, dispHeight, P3D);

  println("Setup Canvas");
  if(!kDrawSkeleton) {
    background(0);
  }
  canvas.stroke(0, 0, 255);
  canvas.strokeWeight(3);
  canvas.smooth();
  println("-- Canvas Setup Complete");

  // setup Kinect tracking
  println("Setup OpenNI");
  setupOpenNI();
  setupOpenNI_CameraImageMode();

  //  setup Kinect Projector Toolkit
  //  kpc = new KinectProjectorToolkit(this, kinect.depthWidth(), kinect.depthHeight());
  //  kpc.loadCalibration("/Users/emilio/Documents/Processing/libraries/KinectProjectorToolkit/examples/CALIBRATION/calibration.txt");

  // setup OSC
  println("Setup OSC");
  setupOSC();

  eyeList.add(new Eye((int)(Math.random() * dispWidth), (int)(Math.random() * dispHeight), 150));
}

void draw() {
  // Update the cam
  kinect.update();

  canvas.beginDraw();
  //background(0);

  // draw the skeleton if true
  if(kDrawSkeleton) {
    // Draw depth image from Kinect
    OpenNI_DrawCameraImage();
    int[] userList = kinect.getUsers();
    for (int i=0; i<userList.length; i++) {
      if (kinect.isTrackingSkeleton(userList[i])) {
        canvas.stroke(userClr[ (userList[i] - 1) % userClr.length ] );
        drawSkeleton(userList[i]);
        if (userList.length == 1) {
          sendOSCSkeleton(userList[i]);
          getJointCoordinates(userList[i]);
        }
      }      
      // draw the center of mass
      if (kinect.getCoM(userList[i], com)) {
        kinect.convertRealWorldToProjective(com, com2d);
        canvas.stroke(100, 255, 0);
        canvas.strokeWeight(1);
        canvas.beginShape(LINES);
        canvas.vertex(com2d.x, com2d.y - 5);
        canvas.vertex(com2d.x, com2d.y + 5);
        canvas.vertex(com2d.x - 5, com2d.y);
        canvas.vertex(com2d.x + 5, com2d.y);
        canvas.endShape();
        canvas.fill(0, 255, 100);
        canvas.text(Integer.toString(userList[i]), com2d.x, com2d.y);
      }
    }
  } 
  else {
//    println("Draw function begin");
    int[] userList = kinect.getUsers();
//    println("Draw function initialized userList[]");
    for (int i=0; i<userList.length; i++) {
//      println("Draw function enters for loop...");
//      println("Index:\t" + Integer.toString(i));
      if (kinect.getCoM(userList[i], com)) {
        kinect.convertRealWorldToProjective(com, com2d);
//        println("Convert Real World To Projective executed");
      }
      
      if (userList.length == 1) {
        sendOSCSkeleton(userList[i]);
//        println("Osc skeleton sent");
      }


    }
    for(int k=0; k<eyeList.size(); k++) {
      Eye newEye;
      newEye = eyeList.get(k);
      newEye.update((int)com2d.x, (int)com2d.y);
      newEye.displayRealEye();
    }
  }

  canvas.endDraw();
  if(kDrawSkeleton) {
    image(canvas, 0, 0); 
  }
}

// --------------------------------------------------------------------------------
//  DRAW SKELETON
// --------------------------------------------------------------------------------

// draw the skeleton with the selected joints
void drawSkeleton(int userId) {
  // println("DRAWING SKELETON");
  canvas.stroke(255, 255, 255, 255);
  canvas.strokeWeight(3);

  drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

// --------------------------------------------------------------------------------
//  DRAW LIMBS
// --------------------------------------------------------------------------------

void drawLimb(int userId, int jointType1, int jointType2) {
  float  confidence;

  // draw the joint position
  PVector a_3d = new PVector();
  confidence = kinect.getJointPositionSkeleton(userId, jointType1, a_3d);
  PVector b_3d = new PVector();
  confidence = kinect.getJointPositionSkeleton(userId, jointType2, b_3d);

  PVector a_2d = new PVector();
  kinect.convertRealWorldToProjective(a_3d, a_2d);
  PVector b_2d = new PVector();
  kinect.convertRealWorldToProjective(b_3d, b_2d);

  canvas.line(a_2d.x, a_2d.y, b_2d.x, b_2d.y);
}

void onNewUser(SimpleOpenNI curContext, int userId)
{
//  println("onNewUser - userId: " + userId);
//  println("\tstart tracking skeleton");

  curContext.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
//  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
{
  //println("onVisibleUser - userId: " + userId);
}

void keyPressed()
{
  switch(key)
  {
  case ' ':
    kinect.setMirror(!kinect.mirror());
//    println("Switch Mirroring");
    break;
  }
}

PGraphics canvas;
color[] userClr = new color[]
{
  color(255, 0, 0), 
  color(0, 255, 0), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};

// --------------------------------------------------------------------------------
//  CAMERA IMAGE SENT VIA SYPHON
// --------------------------------------------------------------------------------
int kCameraImage_RGB = 1;                // rgb camera image
int kCameraImage_IR = 2;                 // infra red camera image
int kCameraImage_Depth = 3;              // depth without colored bodies of tracked bodies
int kCameraImage_User = 4;               // depth image with colored bodies of tracked bodies

int kCameraImageMode = kCameraImage_User; // << Set thie value to one of the kCamerImage constants above

// --------------------------------------------------------------------------------
//  OPENNI (KINECT) SUPPORT
// --------------------------------------------------------------------------------

SimpleOpenNI kinect;

private void setupOpenNI() {
  kinect = new SimpleOpenNI(this);
  if (kinect.isInit() == false) {
//    println("Can't init SimpleOpenNI, maybe the camera is not connected?");
    exit();
    return;
  }

  // Enable depthMapgeneratation
  kinect.enableDepth();
  kinect.enableUser();

  // Disable mirror
  kinect.setMirror(true);
}

// --------------------------------------------------------------------------------
//  OSC SUPPORT
// --------------------------------------------------------------------------------

OscP5 oscP5;
NetAddress oscDestinationAddress;
int oscTransmitPort = 8000;
int oscListenPort = 9002;

private void setupOSC() {

  // initialize OSC support, listen on post oscTransmitPort
  oscP5 = new OscP5(this, oscListenPort);
  oscDestinationAddress = new NetAddress("127.0.0.1", oscTransmitPort);
}

void oscEvent(OscMessage theOscMessage) {
  int classPrediction;
  float likelihood;
  
//  println("### received an osc message. with address pattern "+theOscMessage.addrPattern());
//  println(Integer.toString(theOscMessage.get(0).intValue()) + "\t" + Float.toString(theOscMessage.get(1).floatValue()));
  
  if(theOscMessage.checkAddrPattern("/Prediction") == true && !kDrawSkeleton) {
    classPrediction = theOscMessage.get(0).intValue();
    likelihood = theOscMessage.get(1).floatValue();
//    println("ClassPrediction/MRC:\t" + Integer.toString(classPrediction) + "/" + Integer.toString(MRC));
    if(true) {
      if(MRC != classPrediction) {
        int x_coor = (int) (Math.random() * (dispWidth-100));
        int y_coor = (int) (Math.random() * (dispHeight-100));
        eyeList.add(new Eye(x_coor, y_coor, 200));
        MRC = classPrediction;
      }
    }
//    if(classPrediction == 3 && likelihood > 0.9) {
//      if(MRC != classPrediction) {
//        println("TRIGGER ANIMATION");
//        int x_coor = (int) (Math.random() * dispWidth);
//        int y_coor = (int) (Math.random() * dispHeight);
//        eyeList.add(new Eye(x_coor, y_coor, 100));
//        MRC = classPrediction;
//      }
//    }
//    if(classPrediction == 7 && likelihood > 0.9) {
//      if(MRC != classPrediction) {
//        println("TRIGGER ANIMATION");
//        int x_coor = (int) (Math.random() * dispWidth);
//        int y_coor = (int) (Math.random() * dispHeight);
//        eyeList.add(new Eye(x_coor, y_coor, 100));
//        MRC = classPrediction;
//      }
//    }

  }
}

private void sendOSCSkeletonPosition(String inAddress, int inUserID, int inJointType) {

  // Create the OSC Message with target address
  OscMessage msg = new OscMessage(inAddress);

  PVector p = new PVector();

  float confidence = kinect.getJointPositionSkeleton(inUserID, inJointType, p);

  // Add coordinates to the message
  msg.add(p.x);
  msg.add(p.y);
  msg.add(p.z);

  // Send OSC message
  oscP5.send(msg, oscDestinationAddress);
}

private void sendOSCSkeleton(int inUserID) {

  // Send OSC message for each joint
  sendOSCSkeletonPosition("/head", inUserID, SimpleOpenNI.SKEL_HEAD);
  sendOSCSkeletonPosition("/neck", inUserID, SimpleOpenNI.SKEL_NECK);
  sendOSCSkeletonPosition("/torso", inUserID, SimpleOpenNI.SKEL_TORSO);

  sendOSCSkeletonPosition("/left_shoulder", inUserID, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  sendOSCSkeletonPosition("/left_elbow", inUserID, SimpleOpenNI.SKEL_LEFT_ELBOW);
  sendOSCSkeletonPosition("/left_hand", inUserID, SimpleOpenNI.SKEL_LEFT_HAND);

  sendOSCSkeletonPosition("/right_shoulder", inUserID, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  sendOSCSkeletonPosition("/right_elbow", inUserID, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  sendOSCSkeletonPosition("/right_hand", inUserID, SimpleOpenNI.SKEL_RIGHT_HAND);

  sendOSCSkeletonPosition("/left_hip", inUserID, SimpleOpenNI.SKEL_LEFT_HIP);
  sendOSCSkeletonPosition("/left_knee", inUserID, SimpleOpenNI.SKEL_LEFT_KNEE);
  sendOSCSkeletonPosition("/left_foot", inUserID, SimpleOpenNI.SKEL_LEFT_FOOT);

  sendOSCSkeletonPosition("/right_hip", inUserID, SimpleOpenNI.SKEL_RIGHT_HIP);
  sendOSCSkeletonPosition("/right_knee", inUserID, SimpleOpenNI.SKEL_RIGHT_KNEE);
  sendOSCSkeletonPosition("/right_foot", inUserID, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

private void setupOpenNI_CameraImageMode() {
  println("kCameraImageMode " + kCameraImageMode);

  switch (kCameraImageMode) {
  case 1: // kCameraImage_RGB:
    kinect.enableRGB();
    println("enable RGB");
    break;
  case 2: // kCameraImage_IR:
    kinect.enableIR();
    println("enable IR");
    break;
  case 3: // kCameraImage_Depth:
    kinect.enableDepth();
    println("enable Depth");
    break;
  case 4: // kCameraImage_User:
    kinect.enableUser();
    println("enable User");
    break;
  }
}

private void OpenNI_DrawCameraImage() {
  switch (kCameraImageMode) {
  case 1: // kCameraImage_RGB:
    canvas.image(kinect.rgbImage(), 0, 0);
    // println("draw RGB");
    break;
  case 2: // kCameraImage_IR:
    canvas.image(kinect.irImage(), 0, 0);
    // println("draw IR");
    break;
  case 3: // kCameraImage_Depth:
    canvas.image(kinect.depthImage(), 0, 0);
    //println("draw DEPTH");
    break;
  case 4: // kCameraImage_User:
    canvas.image(kinect.userImage(), 0, 0);
    // println("draw DEPTH");
    break;
  }
}

private void sendJointCoordinates(float[] data) {
  // Create the OSC Message with target address
  OscMessage msg = new OscMessage("/Data");
  msg.add(data);
  // Send OSC message
  oscP5.send(msg, new NetAddress("127.0.0.1", 5000));
}

private void getJointCoordinates(int inUserID) {
  
  PVector head = new PVector();
  float confidenceH = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_HEAD, head);
  PVector neck = new PVector();
  float confidenceN = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_NECK, neck);
  PVector torso = new PVector();
  float confidenceT = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_HEAD, torso);
  
  PVector leftShoulder = new PVector();
  float confidenceLShoulder = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_SHOULDER, leftShoulder);
  PVector leftElbow = new PVector();
  float confidenceLElbow = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_ELBOW, leftElbow);
  PVector leftHand = new PVector();
  float confidenceLHand = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_HAND, leftHand);
  
  PVector rightShoulder = new PVector();
  float confidenceRShoulder = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_SHOULDER, rightShoulder);
  PVector rightElbow = new PVector();
  float confidenceRElbow = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_ELBOW, rightElbow);
  PVector rightHand = new PVector();
  float confidenceRHand = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_HAND, rightHand); 
  
  PVector leftHip = new PVector();
  float confidenceLHip = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_HIP, leftHip);
  PVector leftKnee = new PVector();
  float confidenceLKnee = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_KNEE, leftKnee);
  PVector leftFoot = new PVector();
  float confidenceLFoot = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_LEFT_FOOT, leftFoot);
  
  PVector rightHip = new PVector();
  float confidenceRHip = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_HIP, rightHip);
  PVector rightKnee = new PVector();
  float confidenceRKnee = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_KNEE, rightKnee);
  PVector rightFoot = new PVector();
  float confidenceRFoot = kinect.getJointPositionSkeleton(inUserID, SimpleOpenNI.SKEL_RIGHT_FOOT, rightFoot);
  
  data[0] = head.x;
  data[1] = head.y;
  data[2] = head.z;
  
  data[3] = neck.x;
  data[4] = neck.y;
  data[5] = neck.z;
  
  data[6] = torso.x;
  data[7] = torso.y;
  data[8] = torso.z;
  
  data[9] = leftShoulder.x;
  data[10] = leftShoulder.y;
  data[11] = leftShoulder.z;
  
  data[12] = leftElbow.x;
  data[13] = leftElbow.y;
  data[14] = leftElbow.z;
  
  data[15] = leftHand.x;
  data[16] = leftHand.y;
  data[17] = leftHand.z;
  
  data[18] = rightShoulder.x;
  data[19] = rightShoulder.y;
  data[20] = rightShoulder.z;
  
  data[21] = rightElbow.x;
  data[22] = rightElbow.y;
  data[23] = rightElbow.z;
  
  data[24] = rightHand.x;
  data[25] = rightHand.y;
  data[26] = rightHand.z;
  
  data[27] = leftHip.x;
  data[28] = leftHip.y;
  data[29] = leftHip.z;
  
  data[30] = leftKnee.x;
  data[31] = leftKnee.y;
  data[32] = leftKnee.z;
  
  data[33] = leftFoot.x;
  data[34] = leftFoot.y;
  data[35] = leftFoot.z;
  
  data[36] = rightHip.x;
  data[37] = rightHip.y;
  data[38] = rightHip.z;
  
  data[39] = rightKnee.x;
  data[40] = rightKnee.y;
  data[41] = rightKnee.z;
  
  data[42] = rightFoot.x;
  data[43] = rightFoot.y;
  data[44] = rightFoot.z;
  
  sendJointCoordinates(data);
}
