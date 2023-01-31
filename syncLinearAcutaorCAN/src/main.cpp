#include <Arduino.h>
#include "actuator.h"
#include "delayTrigger.h"
#include "canHandler.h"
#include <CAN.h>
#include "filter.h"

static const u_int8_t leftMotorPosPin = A1;
static const u_int8_t rightMotorPosPin = A2;

const int16_t strokeLength_mm = 300;
servoCityActuator leftCylinder(40,860,strokeLength_mm); //CAN CHANNEL 2 A2
servoCityActuator rightCylinder(60,890,strokeLength_mm); //CAN CHANNEL 1 A1

canProcessor canHandler;

const int8_t mainLoopTime = 1; //Main loop samples and gets the velcoty and position estimate every 
delayTrigger mainLoop(mainLoopTime);    
delayTrigger torqueLoop(50);     
const int16_t postionLoopTime = 200;
delayTrigger positionLoop(postionLoopTime);

int32_t thisTick = 0;
int16_t positionSetPoint = 100;


float rightPostionErrorOutput = 0;
float leftPositionErrorOutput = 0;

averageCalc rightPosAvg(postionLoopTime/mainLoopTime);
float currentRightAveragePos = 0;
averageCalc leftPosAvg(postionLoopTime/mainLoopTime);
float currentLeftAveragePos = 0;

void setup() 
{
  Serial.begin(115200);


  if(!CAN.begin(500E3))
{
    while(1);
}

  leftCylinder.setTorquePID( 50, 0, 0, 50, -1000, 1000); //Will command a torque to keep a speed
  rightCylinder.setTorquePID( 50, 0, 0, 50, -1000, 1000); //Will command a torque to keep a speed

  leftCylinder.setPositionPID(25 , 0, 0, 50, -1000, 1000); //Will command a torque to keep a speed
  rightCylinder.setPositionPID(25 , 0, 0, 50, -1000, 1000); //Will command a torque to keep a speed
}

void loop() 
{

thisTick = millis();

if(mainLoop.trigger(thisTick))
{
  leftCylinder.getSpeedAndPosition(analogRead(leftMotorPosPin), thisTick);
  rightCylinder.getSpeedAndPosition(analogRead(rightMotorPosPin), thisTick);

  currentRightAveragePos = rightPosAvg.getAverage(rightCylinder.position_mm);
  currentLeftAveragePos = leftPosAvg.getAverage(leftCylinder.position_mm);

}

if( torqueLoop.trigger(thisTick) )
{
  //Look at the left vs right positon and adjust accordingly 

  //If the right is closer than the left to the set point than the right needs to slow down 
  if( abs(positionSetPoint-currentRightAveragePos) > abs(positionSetPoint-currentLeftAveragePos) )
  {
    if(rightPostionErrorOutput > 0)//Postion Command
    {
      rightPostionErrorOutput = rightPostionErrorOutput - abs(currentRightAveragePos-currentLeftAveragePos) * (200/5); //reduce command by a 1000 if at 5mm
    }
    else 
    {
      rightPostionErrorOutput = rightPostionErrorOutput + abs(currentRightAveragePos-currentLeftAveragePos) * (200/5); //reduce command by a 1000 if at 5mm
    }
  }
  //else the left is closer to the set point and the needs to slow down 
  else 
  {
    if(leftPositionErrorOutput > 0)//Postion Command
    {
      leftPositionErrorOutput = leftPositionErrorOutput - abs(currentRightAveragePos-currentLeftAveragePos) * (1000/5); //reduce command by a 1000 if at 5mm
    }
    else 
    {
      leftPositionErrorOutput = leftPositionErrorOutput + abs(currentRightAveragePos-currentLeftAveragePos) * (1000/5); //reduce command by a 1000 if at 5mm
    }
  }

  canHandler.updateGoCommandCAN(leftPositionErrorOutput, rightPostionErrorOutput);

  //Send the go command for 
  CAN.beginPacket(speedID);
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

if( positionLoop.trigger(thisTick))
{
  Serial.print("Left Position: "); Serial.print(leftCylinder.position_mm); Serial.print(" Right Position: "); Serial.print(rightCylinder.position_mm); 
  Serial.print(" Left Error output: "); Serial.print(leftPosAvg.getAverage(leftCylinder.position_mm)); Serial.print(" Right Position output: "); Serial.print(rightPosAvg.getAverage(rightCylinder.position_mm)); 
  Serial.println();

  //Get the position error 
  rightPostionErrorOutput = rightCylinder.setPosition(positionSetPoint, currentRightAveragePos, thisTick, 2);
  leftPositionErrorOutput = leftCylinder.setPosition(positionSetPoint, currentLeftAveragePos, thisTick, 2);
}

}