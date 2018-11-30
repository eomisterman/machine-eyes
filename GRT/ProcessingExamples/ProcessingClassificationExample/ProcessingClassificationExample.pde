/**
Processing Classification Example
Version: Development version 1
Author: Nick Gillian

Info: This sketch demonstrates how to use Processing with the GRT GUI for classification. 
You can find out more information about how to use this code here: 

http://www.nickgillian.com/wiki/pmwiki.php/GRT/GUIProcessing
*/

//Import the P5 OSC library
import oscP5.*;
import netP5.*;
import SimpleOpenNI.*;

//Set the pipeline mode (CLASSIFICATION_MODE or REGRESSION_MODE), the number of inputs and the number of outputs
final int pipelineMode = GRT.CLASSIFICATION_MODE;
final int numJoints = 15;
final int numDimensions = 3;
final int numInputs = 45;
final int numOutputs = 1;

//Set the OSC ports used for the GRT GUI and for Processing
final int guiPort = 5000;
final int processingPort = 5001;

//Create a new GRT instance, this will initialize everything for us and send the setup message to the GRT GUI
GRT grt = new GRT( pipelineMode, numInputs, numOutputs, "127.0.0.1", guiPort, processingPort, true );

// Create SimpleOpenNI instance
SimpleOpenNI kinect;
int kCameraImageMode = 4;

//Create some global variables to hold our data
float[] data = new float[ numInputs ];
float[] targetVector = new float[ numOutputs ];
PFont font;

void setup() {
  size(600,600);
  frameRate(30);
  
  //Load the font
  font = loadFont("SansSerif-48.vlw");
  setupOpenNI();
}

void draw() {
  kinect.update();
  background(0);  
  
  if( !grt.getInitialized() ){
    background(255,0,0);  
    println("WARNING: GRT Not Initalized. You need to call the setup function!");
    return;
  }
  
  //Draw the info text
  grt.drawInfoText(20,20);
  
  //Grab the mouse data and send it to the GRT backend via OSC
//  data[0] = mouseX;
//  data[1] = mouseY;
//  data[2] = jointcoordinte
//  grt.sendData( data );
//  kinect.update();
//  OpenNI_DrawCameraImage();
  
  // Get users if skeleton is available
  int[] userList = kinect.getUsers();
  if(userList.length == 1) {
    grt.sendData(getJointCoordinates(userList[0])); 
  }
  else {
    println("No user identified");
  }

}

void keyPressed() {
  
  switch( key ){
    case 'i':
      grt.init( pipelineMode, numInputs, numOutputs, "127.0.0.1", guiPort, processingPort, true );
      break;
    case '[':
      grt.decrementTrainingClassLabel();
      break;
    case ']':
      grt.incrementTrainingClassLabel();
      break;
    case 'r':
      if( grt.getRecordingStatus() ){
        grt.stopRecording();
      }else grt.startRecording();
      break;
    case 't':
      grt.startTraining();
      break;
    case 's':
      grt.saveTrainingDatasetToFile( "TrainingData.txt" );
      break;
    case 'l':
      grt.loadTrainingDatasetFromFile( "TrainingData.txt" );
      break;
    case 'c':
      grt.clearTrainingDataset();
    break;
    case '1': //Set the classifier as ANBC, enable scaling, enable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ANBC, true, true, 5.0 );
    break;
    case '2'://Set the classifier as ADABOOST, enable scaling, disable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ADABOOST, true, false, 5.0 );
    break;
    default:
      break;
  }
  
}

private void setupOpenNI() {
  kinect = new SimpleOpenNI(this);
  if (kinect.isInit() == false) {
    println("Can't init SimpleOpenNI, maybe the camera is not connected?");
    exit();
    return;   
  }

  // Enable depthMapgeneratation
  kinect.enableDepth();
  kinect.enableUser();

  // Disable mirror
  kinect.setMirror(false);
  
  println("SimpleOpenNI intialized");
}

//private void OpenNI_DrawCameraImage() {
//  switch (kCameraImageMode) {
//  case 1: // kCameraImage_RGB:
//    canvas.image(kinect.rgbImage(), 0, 0);
//    // println("draw RGB");
//    break;
//  case 2: // kCameraImage_IR:
//    canvas.image(kinect.irImage(), 0, 0);
//    // println("draw IR");
//    break;
//  case 3: // kCameraImage_Depth:
//    canvas.image(kinect.depthImage(), 0, 0);
//    //println("draw DEPTH");
//    break;
//  case 4: // kCameraImage_User:
//    canvas.image(kinect.userImage(), 0, 0);
//    // println("draw DEPTH");
//    break;
//  }
//}

private float[] getJointCoordinates(int inUserID) {
  
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
  
  PVector rh2D = new PVector();
  kinect.converRealWorldToProjective(rightHand, rh2D);`
  println("rightHand X: \t" + Float.toString(rh2D.x));  
  
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
  
  return data;
}


