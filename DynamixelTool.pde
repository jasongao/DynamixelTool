
/* --------------------------------------------------------------------------
 * SimpleDynamixel ReadAndSet Tool
 * --------------------------------------------------------------------------
 * Processing Wrapper for the Robotis Dynamixel Servos
 * http://code.google.com/p/simple-dynamixel
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  08/26/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 * prog:  Jason Gao / http://jgao.org/
 * date:  04/09/2014 (m/d/y)
 * ----------------------------------------------------------------------------
 * This tool combines the two simple-dynamixel examples, ServoSetGoal and
 * ServoRead, into one tool.
 * ----------------------------------------------------------------------------
 */

import java.text.DecimalFormat;
import java.util.*; 
import SimpleDynamixel.*;
import processing.serial.*;

Servo servo;
ServoViz[] servoVizList;

int maxSpeed = 500; // till 0X3FF
int baudrate = 10000; // baudrate in bps

PVector lastPos = new PVector(0, 0);
float   lastAngle = 0;

// mx-28
int servoRange = 0xFFF; // mx-28 12bit
int servoDeadAngle = 0;

// ax-12
//int servoRange = 0x3FF; // ax-12 10bit
//int servoDeadAngle = 60;

float vizRadius = 0;
float vizDist = 10;

long startTime = 0;
long totalTime = 0;
long count = 0;
long errors = 0;
long resetCount = 0;

int[] servoList;

DecimalFormat df = new DecimalFormat("0.000");

void setup()
{
  smooth();
  size(800, 600);

  // init the dynamixel library
  String serialDev = "/dev/tty.usbserial-A4012906"; // linux, mac
  servo = new Servo();
  servo.init(this, serialDev, baudrate); // use processing serial
  //servo.init(serialDev, baudrate);     // use c++ serial

  servoList = servo.pingRange(0, 16); // set this higher if necessary
  println("---------------------");
  println("servoList: " + Arrays.toString( servoList ));

  // set all servos to the maximum speed
  for (int i=0; i < servoList.length; i++) {
    servo.setMovingSpeed(servoList[i], maxSpeed);
  }   

  servoVizList = new ServoViz[servoList.length];
  for (int i=0; i < servoVizList.length; i++) {
    // disable the torque
    servo.setTorqueEnable(servoList[i], false);

    // set the return delay time for the status packet
    servo.setDelayTime(servoList[i], 0);	// 2us * x
    //servo.setDelayTime(servoList[i],0xFE);	// default 2us * 0xFE = 2 * 250 = 0.5ms

    // init the servo viz
    servoVizList[i] = new ServoViz(servo, servoList[i], servoRange, servoDeadAngle);
  }

  if (servoVizList.length == 1) {
    vizRadius = (height - 2 * vizDist) * .2f;
  } else {
    vizRadius = (width - 2 * vizDist - (servoVizList.length -1) * vizDist) / servoVizList.length * .5f;
  }
}

void draw() {
  background(0);

  pushMatrix();

  translate(vizRadius + vizDist, height/2);

  for (int i=0; i < servoVizList.length; i++) {

    startTime = System.nanoTime();
    // update the data
    servoVizList[i].update();

    totalTime += System.nanoTime()- startTime;
    count++;

    servoVizList[i].draw(g, vizRadius);

    translate(vizRadius * 2 + vizDist, 0);

    if (servoVizList[i].error() != Servo.DX_ERROR_NO)
      println("Servo Error: " + servoVizList[i].errorStr());
  }
  popMatrix();

  // print out the time consumed by the return
  float timeDif = (float)totalTime / count / 1000.0f / 1000.0f;
  text("Readtime/Servo: " + df.format(timeDif) + " ms", 20, 20);

  if (count >= resetCount) {
    count = 0;
    totalTime = 0;
  }
}

void mousePressed() {
  // translate to the right servoViz coords
  //pushMatrix();
  //translate(vizRadius + vizDist, height/2);
  float currOriginX = vizRadius + vizDist;

  for (int i=0; i < servoVizList.length; i++) {
    PVector center = new PVector(currOriginX, height/2);
    PVector pos = new PVector(mouseX, mouseY);

    // are we on this servoViz's circle?
    if (center.dist(pos) < vizRadius) {
      // set the angle in degrees
      PVector dir = PVector.sub(pos, center);
      lastAngle =  - (atan2(dir.y, dir.x) - PI/2.0);
      if (lastAngle < 0) {
        lastAngle += TWO_PI;
      }

      // convert angle to servo range (10=bit or 12-bit, dependong on model)
      int motorPos = (int)(servoRange / TWO_PI * lastAngle);

      println("Set motor " + i + "\tgoal angle: " + degrees(lastAngle) + "\tgoal value: " + motorPos);

      // set the motor to this angle
      servo.setGoalPosition(servoList[i], motorPos);
      break;
    }

    //translate(vizRadius * 2 + vizDist, 0);
    currOriginX += vizRadius * 2 + vizDist;
  }

  //popMatrix();
}
