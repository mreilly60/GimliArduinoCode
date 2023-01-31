#include "delayTrigger.h"


delayTrigger::delayTrigger(uint32_t cycleTime)
{
    lastTime = 0;
    _cycleTime = cycleTime;
}


bool delayTrigger::trigger(uint32_t currentTime)
{
    if((currentTime - lastTime) > _cycleTime)
    {
        lastTime = currentTime;
        return(true);
    }
    else
    {
        return(false);
    }
}
