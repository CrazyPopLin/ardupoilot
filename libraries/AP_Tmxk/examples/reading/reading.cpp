// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the GCS_MAVLink routing 
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_NavEKF.h>
#include <AP_HAL_Linux.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <AP_BattMonitor.h>
#include <SITL.h>
#include <AP_RangeFinder.h>
#include <AP_Tmxk_LIDARScanner.h>
#include <AP_TmxkWaterManger.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_TmxkWaterManager watermanager;
static AP_Tmxk_LIDARScanner lidarscanner;

void setup(void)
{
    //watermanager.init();
    lidarscanner.init();
}

void loop(void)
{
    //watermanager.update();
    lidarscanner.update();

    //hal.console->printf("The nearest angle=%d\n", lidarscanner.getObstaclePos());
    //hal.console->printf("angle=%d, range=%d\n", watermanager.getWaterUsed(), watermanager.getWaterStatus());
    hal.scheduler->delay(20);
}


AP_HAL_MAIN();
