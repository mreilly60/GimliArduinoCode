#include <inttypes.h>

#define speeedID 0x181L
#define commandID 0x201L
#define accelID 0x301L
#define decelID 0x401L
#define syncID 0x080L
#define sdoResponseID 0x581L

#define SDO_REQUEST_CSS_COMMAND   0x20
#define SDO_REQUEST_CSS_QUERY     0x40

#define SDO_RESPONSE_CSS_QUERY    0x40
#define SDO_RESPONSE_CSS_SUCCESS  0x60
#define SDO_RESPONSE_CSS_ERROR    0x80



typedef enum
{
    SET_ENCODER_COUNTER_COMMAND_CH1,
    SET_ENCODER_COUNTER_COMMAND_CH2,
    SET_ACCELERATION_COMMAND_CH1,
    SET_ACCELERATION_COMMAND_CH2,
    SET_DECELERATION_COMMAND_CH1,
    SET_DECELERATION_COMMAND_CH2,
    SAFETY_STOP_COMMAND,
    EMERGENCY_SHUTDOWN_COMMAND,
    RELEASE_SHUTDOWN_COMMAND,
    READ_FAULT_FLAGS_QUERY,
    READ_ENCODER_SPPED_RPM_CH1,
    READ_ENCODER_SPEED_RPM_CH2,
    READ_ABS_ENCODER_COUNTER_CH1,
    READ_ABS_ENCODER_COUNTER_CH2,
    READ_BATTERY_AMPS_CH1,
    READ_BATTERY_AMPS_CH2,
    READ_FOC_TORQUE_AMPS_CH1,
    READ_FOC_TORQUE_AMPS_CH2,
    NUM_SDO_REQUESTS
} srs_sdoCommandQuery_t;

typedef enum 
{
  BOTH_MOTOR,
  RIGHT_MOTOR_ONLY,
  LEFT_MOTOR_ONLY
} srs_selectMotor_t;

typedef struct
{
    bool command; 
    uint16_t cobID;
    uint8_t numDataBytes;
    uint16_t objectIndex;
    uint8_t objectSubIndex;
} srs_roboteqObjectDic;

const srs_roboteqObjectDic roboteqObjectArray[NUM_SDO_REQUESTS] = {
    {true,  0x601, 4, 0x2003, 0x01},     // SET_ENCODER_COUNTER_COMMAND_CH1
    {true,  0x601, 4, 0x2003, 0x02},     // SET_ENCODER_COUNTER_COMMAND_CH2
    {true,  0x601, 4, 0x2006, 0x01},     // SET_ACCELERATION_COMMAND_CH1
    {true,  0x601, 4, 0x2006, 0x02},     // SET_ACCELERATION_COMMAND_CH2
    {true,  0x601, 4, 0x2007, 0x01},     // SET_DECELERATION_COMMAND_CH1
    {true,  0x601, 4, 0x2007, 0x02},     // SET_DECELERATION_COMMAND_CH2
    {true,  0x601, 1, 0x202C, 0x00},     // SAFETY_STOP_COMMAND
    {true,  0x601, 1, 0x200C, 0x00},     // EMERGENCY_SHUTDOWN_COMMAND
    {true,  0x601, 1, 0x200D, 0x00},     // RELEASE_SHUTDOWN_COMMAND
    {false, 0x601, 2, 0x2112, 0x00},     // READ_FAULT_FLAGS_QUERY
    {false, 0x601, 4, 0x2103, 0x01},     // READ_ENCODER_SPPED_RPM_CH1
    {false, 0x601, 4, 0x2103, 0x02},     // READ_ENCODER_SPPED_RPM_CH2
    {false, 0x601, 4, 0x2104, 0x01},     // READ_ABS_ENCODER_COUNTER_CH1
    {false, 0x601, 4, 0x2104, 0x02},     // READ_ABS_ENCODER_COUNTER_CH2
    {false, 0x601, 2, 0x210C, 0x01},     // READ_BATTERY_AMPS_CH1
    {false, 0x601, 2, 0x210C, 0x02},     // READ_BATTERY_AMPS_CH2
    {false, 0x601, 4, 0x211C, 0x01},     // READ_FOC_TORQUE_AMPS_CH1
    {false, 0x601, 4, 0x211C, 0x02}      // READ_FOC_TORQUE_AMPS_CH2
};

class canProcessor{
  
    public:
        canProcessor();
        void rawToPhys();
        void physToRaw();
        void updateGoCommandCAN(float vr,float vl, float maxRPM, uint8_t gbr);
        void updateAccelCommandCAN(float ar, float al, float wc, uint8_t gbr);
        bool generateSDORequestCAN(bool cmd, uint8_t numDataBytes, uint16_t index, uint8_t subIndex, float data);
        int32_t swap_int32( int32_t val );
        void floatToArrayLittleEndian(float data, uint8_t *p_dataArray);

        uint8_t rawSpeedArray[8]; 
        uint8_t rawCommandArray[8]; 
        uint8_t rawAccelArray[8];
        uint8_t sdoTxDataArray[8];
        uint8_t sdoRxDataArray[8];
        bool newDataFlag;

        typedef struct{
            int32_t physRotPerSec; 
            int32_t physCommand;
        }motorDef;

        motorDef rightMotorCan;
        motorDef leftMotorCan;

    private:
  
};