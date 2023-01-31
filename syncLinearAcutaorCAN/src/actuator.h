#include <inttypes.h>

#define MAX_LINEAR_SPEED_MM_PER_S (8)


class PID
{
    public:
        PID();
        void setPID(float postionGain, float integralGain, float derivitiveGain, float windUpPercent,float min_Output,float max_Output);
        float getControlOutput(float setPoint, float measuredValue, uint32_t thisTick);

        float Kp;
        float Ki;
        float Kd;
        float antiWindUp;
        float minOutput;
        float maxOutput;
        float commandOutput;
        float currentError;


    private:

        int32_t deltaT;
        float integralError;
        float lastError;
        float derivitieError;
        uint32_t lastTime;
};

class servoCityActuator
{
    public:
        servoCityActuator(uint16_t ZERO_OFFSET, uint16_t EXTENT_OFFSET, float STROKE_MM);
        void getSpeedAndPosition(uint16_t adcCount, int32_t thisTime);
        float setTorque(float toqrueSP,  float measuredTorque, int32_t thisTime);
        void setTorquePID(float KP, float KI, float KD, float windUpPercent,float min_Output,float max_Output);
        float setPosition(float positionSP_mm, float postionMeas_mm, int32_t thisTick, int16_t deadBan);
        void setPositionPID(float KP, float KI, float KD, float windUpPercent,float min_Output,float max_Output);
        
        float velocity_mm_per_sec;
        float speedMeasured_mm_per_sec;
        float position_mm;
        PID torquePID;
        PID positionPID;

        float map(float in, float inMin, float inMax, float outMin, float outMax);
        uint16_t zeroOffset;
        uint16_t extendedOffset;
        float strokeMM;
        int32_t lastTime;
        float lastPosition;


    private:


};

class syncCyliners 
{
    public:
        syncCyliners(servoCityActuator leftCyliner, servoCityActuator rightCylinder);
};


class averageCalc
{
    public:
        averageCalc(int8_t samplesToAverage);
        float getAverage(float newValue);

    private:
        int8_t samples;
        float sumOfValues;
        int8_t samplesCounter;
        float currentAverage;
};