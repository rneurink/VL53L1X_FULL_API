#ifndef __VL53L1X_FULL_H
#define __VL53L1X_FULL_H

#include "platform/vl53l1_platform.h"
#include "core/VL53L1_api.h"
#include "core/vl53l1_api_calibration.h"



/**
 * @brief This class acts as a wrapper around the ULD api from ST
 */
class VL53L1X
{
    public:
        VL53L1X(); // Constructor

        

    protected:

    private:
        uint8_t     _i2c_address = 0x52;
};

#endif