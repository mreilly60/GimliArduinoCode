#include <inttypes.h>
#include <math.h>

//Definitions
#define DEFAULT_MAX_LINEAR_VELOCITY_MPS (0.3)
#define DEFAULT_MAX_ANGULAR_VELOCITY_RAD_PER_SEC (0.3)
#define NOMINAL_WHEEL_RADIUS_METERS (0.076)
#define LINEAR_ACCELERATION_LIMIT_MPS2 (0.3)
// this spec comes from the manufacturer datasheet for both the Brother and Nidec motors
#define WHEEL_FULL_SPEED_ROT_PER_MIN (3000.0)
#define SEC_PER_MIN (60.0)
#define ENCODER_COUNTS_PER_REVOLUTION (4096*4)
#define MOTOR_GEARBOX_RATIO (9)
#define M_PI (3.14)

// These represent the limits at which we are allowed to drive the motor
#define MIN_MOTOR_OUTPUT  (-1000)
#define MAX_MOTOR_OUTPUT  (1000)

static const float NOMIMAL_AXLE_LENGTH_METERS = 0.901;
static const double ACCELERATION_LIMIT_ROT_PER_SEC2 = LINEAR_ACCELERATION_LIMIT_MPS2 / (NOMINAL_WHEEL_RADIUS_METERS * 2 * M_PI);
static const float WHEEL_FULL_SPEED_ROT_PER_SEC = (WHEEL_FULL_SPEED_ROT_PER_MIN / SEC_PER_MIN);

class robot
{
    public:
        robot();
        void processMotion(float v, float w,float desiredAccelMPSS);
        double wheelRadius;
        double wheelCircum;
        double linearAccelerationLimit_MPSS;
        float maxRPM;
        double maxRadPerSec;
        double axleLength;
        double encoderCountPerRotation;
        float maxLinearVelocityMPS;
        uint8_t gearBoxRatio;

        struct motorDescriptor
        {
            double angularVelocityRotPerSec;
            float desiredRPM;
            float desiredAccelMPSS;
            float desiredDeccelMPPS;

            struct odometry
            {
                uint32_t lastEncoderTime;
                uint32_t lastEncoderCount;
                uint32_t thisEncoderCount;
                double currentVelocityRotPerSec;
                double setPointVelocityRotPerSec;
                double metersTraveled;
            };

            odometry odometryData;
        };

        struct uniCycleModel
        {
            double vUniCycleMPS;
            double wUniCycleRadPerSec;
            double xPositionMeters;
            double yPositionMeters;
            double thetaDegrees;
            double thetaRad;
        };

        motorDescriptor rightMotor;
        motorDescriptor leftMotor;
        uniCycleModel currentPose;
        uniCycleModel desiredPose;

    private:
        double radToDegree(double rad);
        void updateUniCycle(float v, float w, float a);
        void updateDiffDrive();
        void getDiffAccelDeccel(float v, float leftMotorRPM);

};