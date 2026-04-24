#pragma push_macro("abs")
#undef abs

#include "teensystep4.h"
#include "timerfactory.h"
#include "interfaces.h"
#include "TMR.h"


namespace TS4
{
    void begin(bool useDefaultModule)
    {
        if(useDefaultModule)
        {
            TimerFactory::attachModule(new TMRModule<3>());
        }
    }
}

#pragma pop_macro("abs")