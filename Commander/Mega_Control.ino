// Mega board with control program
// Contain sensor reading for MPU9250 IMU and MyoWare Muscle Sensor
// Use Bluetooth for serial communication 

/* MPU 9250
Advanced_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/

/* Bluetooth
BT VCC to Arduino 5V out. Disconnect before running the sketch
BT GND to Arduino GND
BT RX (through a voltage divider) to Arduino TX1 (pin 18)
BT TX  to Arduino RX1 (no need voltage divider)   (pin 19) */ 

#include "MPU9250.h"

// Variables for MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
float currentZ, currentAY, prevZ, prevAY, A, deltaV;
float Vcurrent, Vprev, Xcurrent, Xprev;
int movement;
double times, curtimes, prevtimes;

// Variables for MyoWare Muscle Sensor
float average, base, MAX;
float preV, preA, voltage;
int count;

// Variable for Bluetooth 
char command;

void setup() {
  // MyoMare muscle sensor 
  preV = analogRead(A0);
  base = 64;  
  preA = 0;
  count = 0;
  
  // Bluetooth 
  initial_BT();
  command = 0;
  
  // MPU9250 sensor
  initial_MPU9250();
}

void loop() {
  // read the MPU9250 sensor
  IMU.readSensor();
  curtimes = millis();
   
  // get data from the MPU9250 sensor 
  getMPU9250(); 

  // get EMG data from the MyoWare Muscle Sensor
  getEMG(); 

  // determine the control command need for motor
  setCommand(); 

  // send command to motor through bluetooth 
  Serial1.write(command);
  // Serial.println(command); 

  prevtimes = curtimes;
}

// Bluetooth initialization 
void initial_BT() {
  // communication with the host computer
  Serial.begin(9600);
  // communication with the BT module on serial1
  Serial1.begin(9600);
  while(!Serial) {}
}

// MPU9250 initialization 
void initial_MPU9250() {
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

// get data from the MPU9250 sensor 
void getMPU9250() {
  currentZ = IMU.getAccelZ_mss();
  currentAY = IMU.getGyroY_rads();
  A = (currentAY + prevAY)/2;                   // a = (a1 + a2)/2
  if (A > -0.15 && A < 0.15) {
    A = 0;
  }
  times = (curtimes-prevtimes)/1000;            //delta t = t2 - t1
  deltaV = A * times;                           // delta v = a* delta t
  Vcurrent = Vprev + deltaV;                    // v = v0 + delta v
  Xcurrent = Vprev*times + (A*times*times)/2;   //delta x = v0t + 1/2(at^2)

  if (abs(currentZ - prevZ) < 0.09 || (deltaV == 0)) {
    Vcurrent = 0;
  }

  // update pervious value 
  prevAY = currentAY;
  prevZ = currentZ;
  Vprev = Vcurrent;
} 

// get EMG data from the MyoWare Muscle Sensor
void getEMG() {
  // read the input on analog pin 0
  int sensorValue = analogRead(A0);
  // the analog reading (which goes from 0 - 1023)
  float voltage = sensorValue;
  average = (voltage + preV)/2;
  if (abs(average - base) <= 30 ) {
    MAX = base;
  } else {
    if (preA > average){
      count++;
      MAX = preA;
    } else if(count == 1){
      MAX = base;
    } else {
      count = 0; 
      if (MAX < average){
        MAX = average;
      }
    }         
  }

  // update pervious value
  preV = voltage;
  // for MyoWare Muscle Sensor testing 
  // Serial.println(MAX);
}

// determine the control command need for motor
void setCommand() {
  if (MAX > 750){
    // send capital letter for hand movement 
    if (Vcurrent > 0) {
      command = 'F';
    } else if (Vcurrent < 0) {
      command = 'B';
    } else {
      command = 'I'; 
    }
  } else {
    // send lowwer-case letter for NO hand movement 
    if (Vcurrent > 0) {
      command = 'f';
    } else if (Vcurrent < 0) {
      command = 'b';
    } else {
      command = 'i'; 
    }
  }
} 


// For MPU9250 testing 
void printData() {
  Serial.print("Average A: ");
  Serial.print(A, 6);
  Serial.print("   ");
  Serial.print("t: ");
  Serial.print(times);
  Serial.print("   ");
  Serial.print("delta V: ");
  Serial.print(deltaV, 6);
  Serial.print("   ");
  Serial.print("current V: ");
  Serial.print(Vcurrent, 6);
  Serial.print("   ");
  Serial.print("current X: ");
  Serial.print(Xcurrent, 6);
  Serial.print("   ");
  Serial.print(currentZ, 6);
  Serial.print("   ");
  Serial.println();
}

// For Bluetooth testing
void printBT() {
  // listen for communication from the BT module and then write it to the serial monitor
  if (Serial1.available()) {  
    command = Serial1.read();
    Serial.write( command ); 
  }
}
