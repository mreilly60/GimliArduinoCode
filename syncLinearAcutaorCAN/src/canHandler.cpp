#include "canHandler.h"
#include <string.h>

canProcessor::canProcessor()
{
    memset(rawSpeedArray, 0, sizeof(rawSpeedArray));
    memset(rawCommandArray, 0, sizeof(rawCommandArray));
    memset(rawAccelArray, 0, sizeof(rawAccelArray));

    memset(sdoTxDataArray, 0, sizeof(sdoTxDataArray));
    memset(sdoRxDataArray, 0, sizeof(sdoRxDataArray));

    newDataFlag = false;

    rightMotorCan.physRotPerSec = 0;
    rightMotorCan.physCommand = 0;

    leftMotorCan.physRotPerSec = 0;
    leftMotorCan.physCommand = 0;
}

int32_t canProcessor::swap_int32( int32_t val )
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF ); 
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

void canProcessor::floatToArrayLittleEndian(float data, uint8_t *p_dataArray)
{
    *p_dataArray = (uint8_t) ((int32_t)data & 0xFF);
    *(p_dataArray+1) = (uint8_t) ((int32_t)data >> 8);
    *(p_dataArray+2) = (uint8_t) ((int32_t)data >> 16);
    *(p_dataArray+3) = (uint8_t) ((int32_t)data >> 24);
}


void canProcessor::updateGoCommandCAN(float leftTorque,float rihtTorque)
{
    //Have to scale the comand to -1000 to 1000 which represents the max RPM

    #if 0
    rawCommandArray[7] = (uint8_t) ((int32_t)vl >> 24);
    rawCommandArray[6] = (uint8_t) ((int32_t)vl >> 16);
    rawCommandArray[5] = (uint8_t) ((int32_t)vl >> 8);
    rawCommandArray[4] = (uint8_t) ((int32_t)vl & 0xFF);

    rawCommandArray[3] = (uint8_t) ((int32_t)vr >> 24);
    rawCommandArray[2] = (uint8_t) ((int32_t)vr >> 16);
    rawCommandArray[1] = (uint8_t) ((int32_t)vr >> 8);
    rawCommandArray[0] = (uint8_t) ((int32_t)vr & 0xFF);
    #endif

    floatToArrayLittleEndian(leftTorque, &rawCommandArray[0]);
    floatToArrayLittleEndian(rihtTorque, &rawCommandArray[4]);  
}

void canProcessor::rawToPhys()
{
    leftMotorCan.physRotPerSec = (int32_t)rawSpeedArray[7] << 24;
    leftMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[6] << 16;
    leftMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[5] << 8;
    leftMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[4] << 0;

    rightMotorCan.physRotPerSec = (int32_t)rawSpeedArray[3] << 24;
    rightMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[2] << 16;
    rightMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[1] << 8;
    rightMotorCan.physRotPerSec |= (int32_t)rawSpeedArray[0] << 0;
}




