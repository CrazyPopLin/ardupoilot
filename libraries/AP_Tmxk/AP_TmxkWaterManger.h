
#ifndef _AP_TMXKWATERMANAGER_
#define _AP_TMXKWATERMANAGER_

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdio.h>

#define DATA_LEN 2

class AP_TmxkWaterManager {

public:

    void init();

    void update();

    int16_t getWaterUsed();
    int16_t getWaterStatus();


private://use-angle   //status-range

    //variables for store data in each lidar cycle reading
    int m_waterUsed;
    int m_waterStatus;


    //variables for serial process in each lidar reading
    bool m_start = false;
    int16_t m_data[DATA_LEN];
    char m_dataBuf[20];    //data buff for store serial data temporarily
    int m_dataCount = 0;   //record each loop data count

};

#endif // _AP_SERIALMANAGER_
