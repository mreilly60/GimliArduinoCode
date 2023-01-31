#include <inttypes.h>
#include "robotClass.h"

robot::robot()
{
    wheelRadius = NOMINAL_WHEEL_RADIUS_METERS;
    wheelCircum = wheelRadius*2*M_PI;
    linearAccelerationLimit_MPSS = LINEAR_ACCELERATION_LIMIT_MPS2;
    maxRPM = WHEEL_FULL_SPEED_ROT_PER_MIN;
    maxRadPerSec = DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC; 
    maxLinearVelocityMPS = DEFAULT_MAX_LINEAR_VELOCITY_MPS;
    axleLength = NOMIMAL_AXLE_LENGTH_METERS;
    gearBoxRatio = MOTOR_GEARBOX_RATIO;
}

void robot::updateUniCycle(float v, float w, float a)
{
    //Here we will update the robot with the desired unicycle model values
    desiredPose.vUniCycleMPS = v;
    desiredPose.wUniCycleRadPerSec = w;
    updateDiffDrive();
    
    //If the angular is positive the right wheel will spin slower and the need to accelerate slower
    if(fpclassify(w) == FP_ZERO || fpclassify(v) == FP_ZERO)
    {
        leftMotor.desiredAccelMPSS = a;
        rightMotor.desiredAccelMPSS = a;
    }
    //If going forward
    else if( desiredPose.vUniCycleMPS > 0)
    {
        if( w > 0 )
        {
            rightMotor.desiredAccelMPSS = a;
            if( !(fpclassify(rightMotor.desiredRPM) == FP_ZERO) )
            {
                leftMotor.desiredAccelMPSS = abs( (leftMotor.desiredRPM/rightMotor.desiredRPM)*rightMotor.desiredAccelMPSS );
            }
            else
            {
                //rightMotor.desiredAccelMPSS = 0;
            }
        }
        else
        {
            leftMotor.desiredAccelMPSS = a;
            if( !(fpclassify(leftMotor.desiredRPM) == FP_ZERO) )
            {
                rightMotor.desiredAccelMPSS = abs( (rightMotor.desiredRPM/leftMotor.desiredRPM)*leftMotor.desiredAccelMPSS );
            }
            else
            {
                //leftMotor.desiredAccelMPSS = 0;
            }
        }
    }
    //Going backwards
    else
    {
        if( w < 0 )
        {
            rightMotor.desiredAccelMPSS = a;
            if( !(fpclassify(rightMotor.desiredRPM) == FP_ZERO))
            {
                leftMotor.desiredAccelMPSS = abs( (leftMotor.desiredRPM/rightMotor.desiredRPM)*rightMotor.desiredAccelMPSS );
            }
            else
            {
                //rightMotor.desiredAccelMPSS = 0;
            }
        }
        else
        {
            leftMotor.desiredAccelMPSS = a;
            if( !(fpclassify(leftMotor.desiredRPM) == FP_ZERO))
            {
            rightMotor.desiredAccelMPSS = abs( (rightMotor.desiredRPM/leftMotor.desiredRPM)*leftMotor.desiredAccelMPSS );
            }
            else 
            {
                //leftMotor.desiredAccelMPSS = 0;
            }
        }
    }

}

void robot::updateDiffDrive()
{
    //Will update the left and right wheel velocities needed
    //postive angular will turn right 
    rightMotor.desiredRPM = ( (2*desiredPose.vUniCycleMPS + desiredPose.wUniCycleRadPerSec*axleLength) / (2*wheelRadius) ) * (60/(2*M_PI));
    leftMotor.desiredRPM =  ( (2*desiredPose.vUniCycleMPS - desiredPose.wUniCycleRadPerSec*axleLength) / (2*wheelRadius) ) * (60/(2*M_PI));

}

void robot::processMotion(float v, float w, float accelMPSS)
{
    updateUniCycle(v, w, accelMPSS);
}

double robot::radToDegree(double rad)
{
    return( rad*180/M_PI );
}
