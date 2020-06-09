#include <SPI.h>
#include <time.h>
#include <stdio.h> 
#include <stdlib.h> 
#include "mcp_can.h"
#include <DualVNH5019MotorShield.h>

// pin for SPI CS on CAN Controller
const int spiCSPin = 53;

// var for CAN ID, byte size of CAN message, and CAN ID extension
unsigned long int id = 0x003;
unsigned int len = 2;
unsigned int ext = 0;

// initializes CAN
MCP_CAN CAN(spiCSPin);

//What manages the whole motors and stuff
DualVNH5019MotorShield motorDriver;

//where the pins of the buttons are connected
const int drivePin = 23, cruisePin = 25, neutralPin = 27, brakePin = 29, reversePin = 31;

//throttle input, used to set a speed
int throttlePin = A3;

//relay connection
const int relayPin = 3;

//brake light 
const int brakeLight = 33;

// var for cruise control button state and CAN signal
int cruiseControlButtonState = 1; // 1 -> off, 0 -> on
int cruiseControlSig = 0b0; // 0b0 -> cruise control off, 0b1 -> cruise control on
int cruiseControlSpeed; //Used to save the speed for cruise control

// var for brake pressed signal
int brakePressSig = 0b0; // 0b0 -> brake not pressed, 0b1 -> brake pressed

// var for speed signal
int speedSig = 0;

//var for gear position signal
int gearPositionSig = 0b10; // 0b00 -> park, 0b01 -> reverse, 0b10 -> neutral, 0b11 -> drive. Default state is neutral

// var passed to motor controller to drive motor speed. Range is -400 to 400.
int rawSpeedOutput = 0;

void setup() {
  Serial.begin(115200);
  motorDriver.init();

  pinMode(relayPin, OUTPUT);
  pinMode(drivePin, INPUT_PULLUP);
  pinMode(neutralPin, INPUT_PULLUP);
  pinMode(cruisePin, INPUT_PULLUP);
  pinMode(brakePin, INPUT_PULLUP);
  pinMode(reversePin, INPUT_PULLUP);
  pinMode(brakeLight, OUTPUT); 

  while(CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)){
    Serial.println("CAN BUS transmitter init failed");
    delay(100);
  }
  Serial.println("CAN BUS transmitter Init OK!");
  
}//end funct setup


/* Function to convert values to other ranges. if isReverse = 1, then the return value is multiplied by -1.
   Returns output conversion, always greater than zero
   It is used here for the setM1SPeed since -400 to 0 is reverse with -400 being fastest. 0 to 400 is normal.
*/
int valueConvert(double input, double inputRange, double outputRange) {
  int output;
  output = ((input*outputRange)/inputRange);
  return output;
}//end func valueConverter

/* ******************* T H I N G S   F O R   T E L E M E T R Y   T E A M ***********************
 *  Functions to update variable states.
 */
 
void updateBrakePressed() {
  int brakePress = digitalRead(brakePin);

  // if brake is pressed, light up brake lights and turn off cruise control
  if (brakePress == LOW) {
    brakePressSig = 0b1;
    digitalWrite(brakeLight,HIGH);
    cruiseControlSig = 0b0;
  }
  
  // if brake is not pressed, turn off brake lights
  else {
    brakePressSig = 0b0;
    digitalWrite(brakeLight,LOW);
  }
  
  //Serial.print("Brake Press Sig: ");
  //Serial.println(brakePressSig);
}

void updateGearPosition() {
  if (digitalRead(reversePin) == LOW && digitalRead(neutralPin) == HIGH && digitalRead(drivePin) == HIGH)
    gearPositionSig = 0b01;
  else if (digitalRead(reversePin) == HIGH && digitalRead(neutralPin) == LOW && digitalRead(drivePin) == HIGH)
    gearPositionSig = 0b10;
  else if (digitalRead(reversePin) == HIGH && digitalRead(neutralPin) == HIGH && digitalRead(drivePin) == LOW)
    gearPositionSig = 0b11;
/*
  //Serial.print("Gear Position: ");
  switch(gearPositionSig) {
    case 0b00:
      //Serial.println('P');
      break;
    case 0b01:
      //Serial.println('R');
      break;
    case 0b10:
      //Serial.println('N');
      break;
    case 0b11:
      //Serial.println('D');
      break;    
    default:
      break;
  }
*/  
}

void updateCruiseControl() {
  int newButtonState = digitalRead(cruisePin);
  if (newButtonState != cruiseControlButtonState) {
    if (newButtonState == LOW) {
      cruiseControlSig = !cruiseControlSig; 
      // if cruise control is newly activated, save cruise control speed
      if (cruiseControlSig == 0b1)
        cruiseControlSpeed = valueConvert(analogRead(throttlePin), 1024, 400);
    }
    cruiseControlButtonState = newButtonState;
  }
  //Serial.print("Cruise Control Sig: ");
  //Serial.println(cruiseControlSig); 
}

void updateSpeed() {
  speedSig = map(rawSpeedOutput, 0, 400, 0, 65);
}

// converts throttle input to raw speed output for motor to use
void throttleToRawSpeed() {
  int throttleInput = analogRead(throttlePin);
  rawSpeedOutput = valueConvert(throttleInput, 1023, 400);    
  //Serial.print("Raw speed: ");
  //Serial.println(rawSpeedOutput);
}

//motor speed is converted from an analog pin range of 0-1023 to what the motor driver library uses, 400.
//To go reverse speed, then use negatives (400)the current speed is the voltage returned by pin A3, the throttle pin, a value between 0-1023
int getMotorSpeed(){
  int rawData = analogRead(A3);
}//end func getMotorSpeed


//This function isnt really too useful since the values are weird, but here it is if you need it.
double getCurrentDraw(){
  return motorDriver.getM1CurrentMilliamps();
}//end func getCurrentDraw

// This function is responsible for CHECKING FOR the possibility of having to order another motor driver. If you see this on the console, stuff went bad.
void stopIfFault(){
  if (motorDriver.getM1Fault())
  {
    //Serial.println("Motor 1 fault");
    while (1);
  }
  if (motorDriver.getM2Fault())
  {
    //Serial.println("Motor 2 fault");
    while (1);
  }
}//end funct stopIfFault

void motorHandler() {
  throttleToRawSpeed();
  stopIfFault();

  // if brake is pressed, stop motors
  if (brakePressSig == 0b1) {
    //Serial.print("\nStopping Motors");
    motorDriver.setM1Brake(400);
  } 
  else {
      if (cruiseControlSig == 0b1) {
        digitalWrite(relayPin, HIGH);
        motorDriver.setM1Speed(cruiseControlSpeed);     
      }
      else {
        if (gearPositionSig == 0b01) {
          digitalWrite(relayPin, HIGH);
          motorDriver.setM1Speed(-rawSpeedOutput);
        }
        else if (gearPositionSig == 0b010) {
          digitalWrite(relayPin, LOW);
          motorDriver.setM1Speed(0);
        }
        else if (gearPositionSig == 0b11) {
          digitalWrite(relayPin, HIGH);
          motorDriver.setM1Speed(rawSpeedOutput);
        }        
      }
    }
}

void sendCANMessage() {
  // place the signal values into each byte
  int byte0 = speedSig;
  if (cruiseControlSig == 0b1)
  {
    int tempSpeed = map(cruiseControlSpeed, 0, 400, 0, 65);
    byte0 = tempSpeed;
  }
  Serial.println(gearPositionSig);
  int byte1 = (brakePressSig << 3) | (cruiseControlSig << 2) | gearPositionSig;
  unsigned char msg[2] = {byte0, byte1};
  //Serial.print("byte0: ");
  //Serial.println(byte0);
  //Serial.print("byte1: ");
  //Serial.println(byte1);

  // send message every 250ms
  CAN.sendMsgBuf(id, ext, len, msg);
  delay(250);  
}

void loop() {
  updateGearPosition();
  updateCruiseControl();
  updateBrakePressed();
  motorHandler();
  updateSpeed();
  sendCANMessage();
}//end func loop
