#include "tillerKinematics.h"
#include <math.h>
#include <arduino.h>

tiller::tiller(float maxSpeed, float maxAngular)
{
    minReading = MIN_READING;
    maxReading = MAX_READING;
    midReading = (maxReading - minReading)/2 + minReading;
    palletJackLength = PALLET_JACK_LENGTH;
    maxAngVelocity = maxAngular;
    maxLinVelocity = maxSpeed;
    tillerState.isReversed = false;
    tillerState.driveEnableCounter =0;
}

void tiller::getTillerState(uint16_t adcReading)
{
    if( adcReading > maxReading)
    {
        tillerState.angle = 90.0;
        tillerState.isRight = false;
    }
    else if( adcReading < minReading)
    {
        tillerState.angle = 90.0;
        tillerState.isRight = true;
    }
    else if( adcReading <= midReading)
    {
        tillerState.angle = (float)-90*(adcReading - minReading)/(midReading - minReading) + 90;
        tillerState.isRight = true;

    }
    else
    {
        tillerState.angle =(float)90*(adcReading - midReading)/(maxReading - midReading);
        tillerState.isRight = false;
    }
}

void tiller::getDesiredSpeed(uint16_t adcReading)
{
    if( adcReading == 0 )
    {
        tillerState.driveEnabled = false;
        tillerState.desiredSpeed = 0;
    }

    else if(adcReading == 1)
    {
        tillerState.desiredSpeed = maxLinVelocity; //Scale the reading to 0-1
        tillerState.driveEnabled = true;
        tillerState.isReversed = false;
    }
    else 
    {
        tillerState.desiredSpeed = maxLinVelocity;//* ( (1.0 * adcReading) / 1023)); //Scale the reading to 0-1
        tillerState.isReversed = true;
        tillerState.driveEnabled = true;
    }
}


void tiller::getCritialAngle()
{
    tillerState.criticalAngle = 90 - (180/MATH_PI) * atan(tillerState.desiredSpeed/(maxAngVelocity*palletJackLength));
}

void tiller::getDesiredTuningRadius()
{
    tillerState.turnRadius = palletJackLength * tan( (90-tillerState.angle)*MATH_PI/180);
}

void tiller::getUnicycleModel()
{
    if( fpclassify(tillerState.angle) == FP_ZERO)
    {
        tillerState.desiredV = tillerState.desiredSpeed;
        tillerState.desiredW = 0;
    }
    //Turing radius figure out the v and w and then the direction 
    else if(tillerState.driveEnabled == true)
    {
        if( tillerState.angle >= tillerState.criticalAngle) //cap the angular speed
        {
            tillerState.desiredW = maxAngVelocity;
            tillerState.desiredV = maxAngVelocity * tillerState.turnRadius;
        }
        else //cap the anular speed
        {
            tillerState.desiredV = tillerState.desiredSpeed;
            tillerState.desiredW = tillerState.desiredV / tillerState.turnRadius;
        }

    }
    else
    {
        tillerState.desiredV = 0;
        tillerState.desiredW = 0;
    }

    if( !(tillerState.isReversed) )  //Driving forward
    {
        if( !tillerState.isRight )
        {
            tillerState.desiredW = -tillerState.desiredW;
        }
    }
    else
    {
        Serial.println("Reversed");
        tillerState.desiredV = -tillerState.desiredV;
        if( tillerState.isRight )
        {
            tillerState.desiredW = -tillerState.desiredW;
        }
    }
}

void tiller::getMovmentCommand(float adcAngle, float adcThrottle)
{
    getTillerState(adcAngle);
    getDesiredSpeed(adcThrottle);
    getCritialAngle();
    getDesiredTuningRadius();
    getUnicycleModel();
}

void tiller::printDebug()
{
    Serial.print(" Turning Radius: "); Serial.print(tillerState.turnRadius);
    Serial.print(" Angle: ");Serial.print(tillerState.angle);
    Serial.print("  Velcoity: ");Serial.print(tillerState.desiredV);
    Serial.print("  Ang Velocity: ");Serial.print(tillerState.desiredW);
    Serial.print("  Throttle Speed: ");Serial.print(tillerState.desiredSpeed);
    Serial.print(" Drive Enabled:"); Serial.print(tillerState.driveEnabled);
}
