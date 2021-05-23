/*
 "compass_rose.pde"
 (c) 2019 lingib
 https://www.instructables.com/member/lingib/instructables/
 Last update: 28 November 2019
 
 --------
 ABOUT
 --------
 This program displays a calibrated compass graticule|rose for displaying the 
 output of the matching arduino "imu_tilt_compass_v1.ino" program that uses an
 Ivensense MPU-9250 9DOF accelerometer|gyro|magnetometer sensor module.
 
 The arduino compass code uses open-source qauternion code by Magwick|Mahoney.
 
 ----------
 COPYRIGHT
 ----------
 This is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License. If 
 not, see <http://www.gnu.org/licenses/>.
 */

import processing.serial.*;
Serial myPort;                                    // create Serial port instance

// ----- data input
String inputString = "";                          // for incoming data 
boolean connected = false;

// ----- graphics
int number = 0;                                   // graticule
float heading = -155.0;                           // degrees
float angle;                                      // convert to radians      

// ----- software timer
int time1 = 10;
int stop1;

// ==========================
// setup()
// ==========================
void setup()
{
  // ----- define drawing area size
  size(600, 600, P3D);                            // P3D allows rotation about the Z axis
  background(0);                                  // initial background color

  // ----- configure the serial port
  printArray(Serial.list()); 
  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.bufferUntil('\n');                      // serialEvent() won't trigger until buffer has "\n"
  myPort.clear();

  // ----- start software timer
  stop1 = millis()+time1;
}

// ==========================
// draw()
// ==========================
void draw() {
  background(255);
  translate(width/2, height/2);
  scale(0.7);

  // ----- rotate compass
  pushMatrix();
  angle = radians(heading);
  rotateZ(angle);
  draw_compass_rose();
  popMatrix();

  // --- draw compass needle
  fill(255, 0, 0);
  triangle(-width*0.05, 0, width*0.05, 0, 0, -height*0.38);
  stroke(0);
  strokeWeight(1);
  fill(255);
  triangle(-width*0.05, 0, width*0.05, 0, 0, height*0.38);

  // ----- debug
  //if (millis() > stop1) {
  //  stop1 += time1;
  //  heading+=1;
  //}
}

// ==========================
// draw_compass_rose()
// ==========================
void draw_compass_rose() {

  pushMatrix();

  // ----- draw compass 
  pushMatrix();
  noStroke();
  fill(200);                       // light grey
  ellipse(0, 0, width, height);
  popMatrix();

  // ----- blank the center of the circle
  pushMatrix();
  fill(255);                       // white
  ellipse(0, 0, width*0.7, height*0.7); 
  popMatrix();

  // ----- draw 10 degree markers
  for (int i=0; i<36; i++) {
    angle = radians(-i*10);
    pushMatrix();
    fill(0);
    rotateZ(angle);
    rect(width*0.38, 0, width*0.01, height*0.008);
    popMatrix();
  }

  // ----- draw 30 degree markers
  for (int i=0; i<12; i++) {
    angle = radians(-i*30);
    pushMatrix();
    fill(0);
    rotateZ(angle);
    rect(width*0.38, 0, width*0.05, height*0.008);
    popMatrix();
  }

  // ----- add NSWE labels
  pushMatrix();
  fill(0);    // Fill color black 
  textSize(50);
  text("N", -15, -width*0.54);
  text("E", width*0.54+5, 20);
  text("S", -15, width*0.54+35);
  text("W", -width*0.54-45, 20);  
  popMatrix();

  // ----- add degrees
  textSize(20);
  fill(0);  
  angle = radians(0);
  rotateZ(angle);
  text("360", -18, -height*0.445);  
  angle = radians(30);
  rotateZ(angle);
  text("30", -10, -height*0.445);  
  angle = radians(30);
  rotateZ(angle);
  text("60", -10, -height*0.445);  
  angle = radians(30);
  rotateZ(angle);
  text("90", -10, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("120", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("150", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("180", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("210", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("240", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("270", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("300", -18, -height*0.445);
  angle = radians(30);
  rotateZ(angle);
  text("330", -18, -height*0.445);

  popMatrix();
}

// =======================
// serial event  (called with each Arduino data string)
// =======================
void serialEvent(Serial myPort) {

  // ----- wait for a line-feed
  inputString = myPort.readStringUntil('\n');

  // ----- validate input data
  if (inputString != null) 
  {

    // ----- establish connection
    inputString = trim(inputString);                   // remove leading/trailing whitespace
    if (connected == false) {
      if (inputString.equals("S")) {
        connected = true;                              // connection established
        myPort.write("S");                             // acknowledge
      }
    } else {
      // ----- rotate compass
      heading = -(float(inputString));                 // update compass rose
      println(-heading);
      myPort.write("S");                               // request next data
    }
  }
}
