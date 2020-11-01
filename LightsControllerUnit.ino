#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <SPI.h>
#include <time.h>
#include <stdio.h> 
#include <stdlib.h> 
#include "mcp_can.h"

// pin for SPI CS on CAN Controller
const int spiCSPin = 10;

// pins for buttons
const int leftTurnButton = 3;
const int rightTurnButton = 7;
const int hazardsButton = 5;
const int headlightsButton = 2;

// var for CAN ID, message byte size, and CAN ID extension
unsigned long int id = 0x004;
unsigned int len = 1;
unsigned int ext = 0;

MCP_CAN CAN(spiCSPin);

void setup() {
  Serial.begin(115200);
  pinMode(leftTurnButton, INPUT_PULLUP);
  pinMode(rightTurnButton, INPUT_PULLUP);
  pinMode(hazardsButton, INPUT_PULLUP);
  pinMode(headlightsButton, INPUT_PULLUP);
  while(CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)){
    Serial.println("CAN BUS transmitter init failed");
    delay(100);
  }
  Serial.println("CAN BUS transmitter Init OK!");
}

void loop() {
  // update signal values
  int leftTurnSig = updateLeftTurn();
  int rightTurnSig = updateRightTurn();
  //int hazardsSig = updateHazards();
  int hazardsSig = 0b0;
  int headlightsSig = updateHeadlights();

  // place signal values into each byte of the CAN message
  int byte0 = (headlightsSig << 3) | (leftTurnSig << 2) | (rightTurnSig << 1) | hazardsSig;
  unsigned char msg[1] = {byte0};
  Serial.print("CAN data: ");
  Serial.println(byte0);
  // send message every 300ms
  CAN.sendMsgBuf(id, ext, len, msg);
  delay(300);
  
}

int updateLeftTurn(){
  int leftTurnSig = 0b0;
  if (digitalRead(leftTurnButton) == HIGH)
    leftTurnSig = 0b1;
  Serial.print("Left Sig: ");
  Serial.println(leftTurnSig);  
  return leftTurnSig;
}

int updateRightTurn(){
  int rightTurnSig = 0b0;
  if (digitalRead(rightTurnButton) == HIGH)
    rightTurnSig = 0b1;
  Serial.print("Right Sig: ");
  Serial.println(rightTurnSig);  
  return rightTurnSig;
}

int updateHazards() {
  int hazardsSig = 0b0;
  if (digitalRead(hazardsButton) == HIGH)
    hazardsSig = 0b1;
  Serial.print("Hazards Sig: ");
  Serial.println(hazardsSig);  
  return hazardsSig;
}

int updateHeadlights() {
  int headlightsSig = 0b0;
  if (digitalRead(headlightsButton) == HIGH)
    headlightsSig = 0b1;
  Serial.print("Headlights Sig: ");
  Serial.println(headlightsSig);  
  return headlightsSig;
}
