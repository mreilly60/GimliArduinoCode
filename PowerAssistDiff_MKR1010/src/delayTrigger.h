#include <intTypes.h>

class delayTrigger
{
    public:
    delayTrigger(uint32_t _cycleTime);
    bool trigger(uint32_t currentTime);
   
    private:
    uint32_t _cycleTime = 1000;
    uint32_t lastTime;
};
