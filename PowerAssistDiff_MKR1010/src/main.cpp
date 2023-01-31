#include <Arduino.h>
#include <delayTrigger.h>
#include <SPI.h>
#include <CAN.h>
#include "canHandler.h"
#include "tillerKinematics.h"
#include "robotClass.h"
#include "filter.h"

#define tillerAnglePin A1
#define forwardPin 0 //D0 goes low going forward  
#define reversePin 1 //D1 goes low going reverse 

delayTrigger mainLoopTimer(1);
delayTrigger goCommandTimer(20);
delayTrigger accelCommandTimer(20);
delayTrigger decelCommandTimer(20);

canProcessor canHandler;

robot gimli;
tiller myTiller(gimli.maxLinearVelocityMPS, gimli.maxRadPerSec);

LowPass angleLPF(50, 1000);
int16_t tillerAngleFilt = 0;

float sumVelcoity = 0;
float sumW = 0;
float averageW = 0;
float averageVelocity = 0;
int8_t samples = 1; //Counts the loops that was run through to get the velcoty

void onReceive(int packetSize)
{
    uint8_t i = 0;
    long packetID;

    packetID = CAN.packetId();
    //Serial.println(packetID,HEX);

    if (packetID == sdoResponseID )
    {
      while (CAN.available()) 
      {
        canHandler.sdoRxDataArray[i] = (uint8_t)CAN.read();
        i++;
      }
      canHandler.newDataFlag = true;
    }
    else if (packetID == speeedID)
    {
      while (CAN.available()) 
      {
        canHandler.rawSpeedArray[i] = (uint8_t)CAN.read();
        i++;
      }
    }
    else
    {
      //Serial.println("unknown ID");
    }


  //Read the potentiometer and convert to an angle 

}


void setup() {

pinMode(forwardPin, INPUT_PULLUP);
pinMode(reversePin,INPUT_PULLUP);

Serial.begin(9600);
delay(10);
if(!CAN.begin(500E3))
{
    while(1);
  Serial.println("Can't start CAN");
}

delay(1000);
digitalWrite(6,HIGH); // turn on main contactor 

//CAN.onReceive(onReceive);  // register the receive callback
}

int16_t throttleInput = 510;

void loop() 
{

static float thisTick = 0;
thisTick = millis();

if(mainLoopTimer.trigger(thisTick))
{  
  if( !digitalRead(forwardPin) && digitalRead(reversePin) ) 
  {
  throttleInput = 1; 
  }
  else if( digitalRead(forwardPin) && !digitalRead(reversePin) )
  {
  throttleInput = -1;
  }
  else 
  {
    throttleInput = 0;
  }

  //Serial.print("trhottle Input: ");Serial.print(throttleInput);Serial.println();

  tillerAngleFilt = angleLPF.filt(analogRead(tillerAnglePin));
  myTiller.getMovmentCommand(tillerAngleFilt, throttleInput);
  myTiller.printDebug();

  sumVelcoity+=myTiller.tillerState.desiredV;
  sumW+=myTiller.tillerState.desiredW;
  samples++;
  if(samples >= 10)
  {
    averageW = sumW/samples;
    averageVelocity = sumVelcoity/samples;
    samples = 0;
    sumVelcoity = 0;
    sumW = 0;
  }

  gimli.processMotion(averageVelocity, averageW, gimli.linearAccelerationLimit_MPSS); //Set the speeds to 0
  canHandler.updateGoCommandCAN(gimli.rightMotor.desiredRPM, gimli.leftMotor.desiredRPM, gimli.maxRPM, gimli.gearBoxRatio); 
  canHandler.updateAccelCommandCAN(gimli.rightMotor.desiredAccelMPSS, gimli.leftMotor.desiredAccelMPSS, gimli.wheelCircum, gimli.gearBoxRatio); 

  Serial.print(analogRead( tillerAnglePin));Serial.print(" Right Acceleration: ");Serial.print(gimli.rightMotor.desiredAccelMPSS);Serial.print(" Right RPM: ");Serial.print(gimli.rightMotor.desiredRPM);
  Serial.print(" Left Acceleration: ");Serial.print(gimli.leftMotor.desiredAccelMPSS);Serial.print(" Left RPM: ");Serial.print(gimli.leftMotor.desiredRPM);
  Serial.println();
}

if(goCommandTimer.trigger(thisTick))
{
    //Send the go command for 
    CAN.beginPacket(commandID);
    CAN.write(canHandler.rawCommandArray[0]);
    CAN.write(canHandler.rawCommandArray[1]);
    CAN.write(canHandler.rawCommandArray[2]);
    CAN.write(canHandler.rawCommandArray[3]);
    CAN.write(canHandler.rawCommandArray[4]);
    CAN.write(canHandler.rawCommandArray[5]);
    CAN.write(canHandler.rawCommandArray[6]);
    CAN.write(canHandler.rawCommandArray[7]);
    CAN.endPacket();
}

if(accelCommandTimer.trigger(thisTick))
{
      //Send the go command for 
    CAN.beginPacket(accelID);
    CAN.write(canHandler.rawAccelArray[0]);
    CAN.write(canHandler.rawAccelArray[1]);
    CAN.write(canHandler.rawAccelArray[2]);
    CAN.write(canHandler.rawAccelArray[3]);
    CAN.write(canHandler.rawAccelArray[4]);
    CAN.write(canHandler.rawAccelArray[5]);
    CAN.write(canHandler.rawAccelArray[6]);
    CAN.write(canHandler.rawAccelArray[7]);
    CAN.endPacket();
}

if(decelCommandTimer.trigger(thisTick))
{
      //Send the go command for 
    CAN.beginPacket(decelID);
    CAN.write(canHandler.rawAccelArray[0]);
    CAN.write(canHandler.rawAccelArray[1]);
    CAN.write(canHandler.rawAccelArray[2]);
    CAN.write(canHandler.rawAccelArray[3]);
    CAN.write(canHandler.rawAccelArray[4]);
    CAN.write(canHandler.rawAccelArray[5]);
    CAN.write(canHandler.rawAccelArray[6]);
    CAN.write(canHandler.rawAccelArray[7]);
    CAN.endPacket();
}

}