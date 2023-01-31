//This will convert the tiller reading into a desired v,w

#include <inttypes.h>

#define MIN_READING (590) //Corresponds to 90deg
#define MAX_READING (870) //Corresponds to -90deg

#define PALLET_JACK_LENGTH (1.52) //Used to translate turn radius into 

#define MATH_PI (3.14)

class tiller
{
    public:
        tiller(float maxSpeed, float maxAngular);
        void getTillerState(uint16_t adcReading);
        void getDesiredSpeed(uint16_t adcReading);
        void getCritialAngle();
        void getDesiredTuningRadius();
        void getUnicycleModel();
        void getMovmentCommand(float adcAngle, float adcThrottle);
        void printDebug();

        uint16_t minReading;
        uint16_t maxReading;
        uint16_t midReading;
        float maxLinVelocity;
        float maxAngVelocity;
        float palletJackLength;

        typedef struct 
        {
            float angle;
            bool isRight; //Determine if need to turn left or right
            float desiredV;
            float desiredW;
            float desiredAccel;
            float criticalAngle;
            float desiredSpeed;
            float turnRadius;
            bool driveEnabled;
            bool isReversed;
            int8_t driveEnableCounter;
        }tillerDescriptor;

    tillerDescriptor tillerState;        
};