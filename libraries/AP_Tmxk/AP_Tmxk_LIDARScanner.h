#pragma once


#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdio.h>
#include <array>

#define LIDAR_STREAM 360
#define UPPER 250
#define LOWER 100
#define RECEIVED_BYTE_MAX 5 //temporary store coming lidarscanner data format for each reading
#define RECEIVED_BYTE_LEN 4// always 4 bytes, 2 bytes for angle and 2 bytes for distance
#define UPDATE_OBSTACLE_MAX 9//有几种情况误差就是累加到几？？1）角度偏移，2）角度没有测量到，3）
#define UPDATE_OBSTACLE_MIN 0

//#define lidar_stream_param_default 100

class AP_Tmxk_LIDARScanner {


private:

    //variables for store data in each lidar cycle reading
    struct{
        int AngleArr[LIDAR_STREAM];
        int RangeArr[LIDAR_STREAM];
        bool isObstacle[LIDAR_STREAM/2+1] = {};  //1: unsafe; 0:safe;both 1 and 180 degree are accessible
        bool available = false;

        uint16_t StreamCount  = 0;
        uint16_t ErrorCount = 0;

    }scan;

    bool m_pilotCommand;

    //AP_Int16 lidar_stream_parameter;              /// parameter about to setup lidar streams

    struct{
        char received_Byte[RECEIVED_BYTE_MAX];
        uint8_t count = 0;
        uint8_t error = 0;
        bool recvInProgress =false;
        bool recvFirst = false;
        uint16_t angle = 0;
        uint16_t range = 0;
    }stream;

    uint32_t monitor_time=0;

    bool isScannerWork = true;

    bool isSafe = true;

    uint8_t buff_filter = UPDATE_OBSTACLE_MIN ;
public:

    AP_Tmxk_LIDARScanner();

    void init();

    void update();

    void checkObstacle();

    enum Data_Error_Type{
        No_Error_Type    =  0,
        Serial_Miss_Data =  1,  //miss some data byte maybe angle or range in one stream loop
        Serial_Miss_End  =  2,
        Range_Error_Read =  3,
    };

    //get obstacle info into the reference array from AP_VFH class, and update to the max 5 at same time
    bool getObstacle(uint8_t (&array)[LIDAR_STREAM/2+1]);

    bool ScanAvailable() const { return scan.available; }

    bool LidarScannerAvailable() const { return isScannerWork; }

    int16_t getPilotCommand() const { return m_pilotCommand; }

    int angleTranfer(int _angle) { return (_angle  >= 180 && _angle <=360 ) ? _angle-180 : -1; }

    bool checkSafe() const { return isSafe; }

};


