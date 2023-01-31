#include "actuator.h"
#include <arduino.h>

servoCityActuator::servoCityActuator(uint16_t ZERO_OFFFSET, uint16_t EXTEND_OFFSET, float STROKE_MM)
{
    zeroOffset = ZERO_OFFFSET;
    extendedOffset = EXTEND_OFFSET;
    strokeMM = STROKE_MM;
}

void servoCityActuator::getSpeedAndPosition(uint16_t adcCount, int32_t thisTime)
{
    //Add sliding dead ban   
    position_mm = map(adcCount, zeroOffset, extendedOffset, 0, strokeMM);
    if( !( abs((position_mm-lastPosition)) > 0.5) ) 
    {
        //Don't update the positin and if it hasn't moved the velocity is zero
        position_mm = lastPosition;
    }
    velocity_mm_per_sec = (position_mm - lastPosition) / (((float)thisTime - lastTime)/1000);
    lastPosition = position_mm;
    lastTime = thisTime;
}


float servoCityActuator::map(float in, float inMin, float inMax, float outMin, float outMax)
{
    return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}  

void servoCityActuator::setTorquePID(float KP, float KI, float KD, float windUpPercent,float min_Output,float max_Output)
{
    torquePID.setPID(KP, KI, KD, windUpPercent, min_Output, max_Output);
}

void servoCityActuator::setPositionPID(float KP, float KI, float KD, float windUpPercent,float min_Output,float max_Output)
{
    positionPID.setPID(KP, KI, KD, windUpPercent, min_Output, max_Output);
}

float servoCityActuator::setTorque(float toqrueSP, float torqueMeasured, int32_t thisTime)
{
    return( torquePID.getControlOutput(toqrueSP, torqueMeasured, thisTime) );
}

float servoCityActuator::setPosition(float positionSP_mm, float postionMeas_mm, int32_t thisTick, int16_t deadBan)
{
    if( abs( postionMeas_mm-positionSP_mm) > deadBan)
    {
        return( positionPID.getControlOutput(positionSP_mm, postionMeas_mm, thisTick) );
    }
    else 
    {
        return(0);
    }
}


PID::PID()
{
    currentError = 0;
    integralError = 0;
    derivitieError = 0;
}

void PID::setPID(float postionGain, float integralGain, float derivitiveGain, float windUpPercent,float min_Output,float max_Output)
{
    Kp = postionGain;
    Ki = integralGain;
    Kd = derivitiveGain;
    antiWindUp = windUpPercent;
    minOutput = min_Output;
    maxOutput = max_Output; 
}

float PID::getControlOutput(float setPoint, float measuredValue, uint32_t thisTick)
{
    //Proportional error
    currentError = setPoint - measuredValue;

    //Integral error
    integralError += currentError;
    if (integralError >= maxOutput) 
    {
        integralError = (antiWindUp * maxOutput ) / 100;
    }
    else if (integralError <= minOutput)
    {
        integralError = ( antiWindUp * minOutput) / 100;
    }

    derivitieError = currentError - lastError;

    deltaT = thisTick - lastTime;

    commandOutput = Kp*currentError + (Ki*deltaT)*integralError + (Kd/deltaT)*derivitieError; //PID control compute
    if (commandOutput >= maxOutput)
    {
        commandOutput = maxOutput;
    }
    else if (commandOutput <= minOutput)
    {
        commandOutput = minOutput;
    } 

    lastError = currentError;
    lastTime = thisTick;

    return(commandOutput);
}

averageCalc::averageCalc(int8_t samplesToAverage)
{
    samples = samplesToAverage;
    sumOfValues = 0;
    currentAverage = 0;
}

float averageCalc::getAverage(float newValue)
{
    samplesCounter++;
    sumOfValues+=newValue;

    if(samplesCounter>samples)
    {   
        currentAverage = sumOfValues/samplesCounter;
        samplesCounter = 0;
        sumOfValues = 0;
    }

    return( currentAverage );
}