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

void canProcessor::updateAccelCommandCAN(float ar, float al, float wc, uint8_t gbr)
{
    //Gets sent in 0.1*RPM/s
    floatToArrayLittleEndian((ar/wc)*10*gbr*60, &rawAccelArray[0]);
    floatToArrayLittleEndian((al/wc)*10*gbr*60, &rawAccelArray[4]);  
}

void canProcessor::updateGoCommandCAN(float vr, float vl, float maxRPM, uint8_t gbr)
{
    //Have to scale the comand to -1000 to 1000 which represents the max RPM
/*
    if (vr > 0.0)
    {
        vr = (vr>maxRPM)?maxRPM:vr;
    }
    else
    {
        vr = (vr<maxRPM*(-1))?maxRPM*(-1):vr;
    }

    if (vl > 0.0)
    {
        vl = (vl>maxRPM)?maxRPM:vl;
    }
    else
    {
        vl = (vl<maxRPM*(-1))?maxRPM*(-1):vl;
    }
*/
    vr = gbr * (vr/maxRPM) * 1000;
    vl = gbr * (vl/maxRPM) * 1000;
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

    floatToArrayLittleEndian(vr, &rawCommandArray[0]);
    floatToArrayLittleEndian(vl, &rawCommandArray[4]);  
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

bool canProcessor::generateSDORequestCAN(bool cmd, uint8_t numDataBytes, uint16_t index, uint8_t subIndex, float data)
{
    bool success = false;
    uint8_t css;   // Client Command Specifier
    uint8_t bit2;
    uint8_t bit3;


    if ((numDataBytes > 0) && (numDataBytes < 5))
    {
        success = true;

        css = cmd?SDO_REQUEST_CSS_COMMAND:SDO_REQUEST_CSS_QUERY;

        numDataBytes = numDataBytes & 0x03;
        numDataBytes = numDataBytes << 2;
        bit2 = numDataBytes & 0x04;
        bit3 = numDataBytes & 0x08;

        sdoTxDataArray[0] = css | (bit3 ^ (bit2<<1)) | bit2;
        sdoTxDataArray[1] = index & 0x00FF;
        sdoTxDataArray[2] = index >> 8;
        sdoTxDataArray[3] = subIndex;
        floatToArrayLittleEndian(data, &sdoTxDataArray[4]);
    }

    return success;
}


