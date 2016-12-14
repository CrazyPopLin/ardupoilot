
#pragma once


#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdio.h>

#include "AP_Tmxk_LIDARScanner.h"


class AP_Tmxk_VFH {


private:

    //bool m_pilotCommand_;

    const AP_Tmxk_LIDARScanner &_lidarscanner;

public:

    // Constructor
    AP_Tmxk_VFH(const AP_Tmxk_LIDARScanner &_lidarscanner);

    void init();

    void update();

};
