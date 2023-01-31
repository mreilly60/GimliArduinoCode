#include <inttypes.h>

#define speedID 0x201L

class canProcessor{
  
    public:
        canProcessor();
        void rawToPhys();
        void physToRaw();
        void updateGoCommandCAN(float leftTorque,float rihtTorque);
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