// Uno board with motor control program, direatly connect to robot arm
// Contain MPU9250 IMU sensor for stablization 
// Use Bluetooth for serial communication 

/* Bluetooth
//  BT VCC to Arduino 5V out. Disconnect before running the sketch
//  BT GND to Arduino GND
//  BT RX (through a voltage divider) to Arduino pin 5
//  BT TX  to Arduino pin 4 (no need voltage divider) */ 

#include <SoftwareSerial.h>

// Variables for Bluetooth 
SoftwareSerial BTserial(4, 5); // RX,TX 
char input; 

int count, arm, hand; 

void setup() {
  // Bluetooth 
  initial_BT();
  input = 0;

  // Motor 
  initial_Motor(); 
  count = 0; 
  arm = 0; 
  hand = 0; 
}

void loop() {
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (BTserial.available()) {
    input = BTserial.read();
    Serial.println(input); 
  }
  
  // control hand motor using bluetooth input 
  handControl();
  
  // control arm motor using bluetooth input 
  armControl();
  
  
  if (count == 30) {
    armMotor(); 
    handMotor(); 
    count = 0; 
  }

  count++; 
}

// Bluetooth initialization 
void initial_BT() {
  Serial.begin(9600);
  BTserial.begin(9600);
  while(!Serial) {}
  // pin for mode switching 
}

// Motor initialization 
void initial_Motor() {
  // Arm motor control pin
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  // Hand motor control pin
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  // inital status of motor 
  armDoNothing();
  handDoNothing();
}

// control the arm motor base on the input command
void armControl() {
  if (input == 'f' || input == 'F') {
    arm++; 
  } else if (input == 'b' || input == 'B') {
    arm--;
  } 
} 

// control the arm motor base on the input command
void armMotor() {
  if (arm > 0) {
    armForward();
  } else if (arm < 0 ) {
    armBack();
  } else {
    armDoNothing();
  }
  arm = 0; 
} 

// control the hand motor base on the input command
void handControl() {
  if (input <= 'a') {
    hand++; 
  } 
} 

// control the hand motor base on the input command
void handMotor() {
  if (hand > 0) {
    handForward();
  } else {
    handDoNothing();
  }
  hand = 0; 
} 

// simple arm motor control command 
void armDoNothing() {
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
}
void armBack() {
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
}
void armForward() {
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
}

// simple hand motor control command 
void handDoNothing() {
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
}
void handForward() {
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}
