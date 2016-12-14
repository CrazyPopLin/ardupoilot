#pragma once


#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdio.h>
#include <array>

#define LIDAR_STREAM 360
#define UPPER 250
#define LOWER 50
#define RECEIVED_DATA_MAX 8 //temporary store coming lidarscanner data format for each reading
#define RECEIVED_DATA_LEN 4// always 4 bytes, 2 bytes for angle and 2 bytes for distance

//#define lidar_stream_param_default 100

class AP_Tmxk_LIDARScanner {


private:

    //variables for store data in each lidar cycle reading
    struct{
        int AngleArr[LIDAR_STREAM];
        int RangeArr[LIDAR_STREAM];
        bool isObstacle[LIDAR_STREAM/2] = {};  //1: unsafe; 0:safe
        bool available = false;

        uint16_t StreamCount  = 0;
        uint16_t ErrorCount = 0;

    }scan;

    bool m_pilotCommand;

    //AP_Int16 lidar_stream_parameter;              /// parameter about to setup lidar streams

    struct{
        char received_Byte[RECEIVED_DATA_MAX];
        uint8_t count = 0;
        uint8_t error = 0;
        bool recvInProgress =false;
        bool recvFirst = false;
        uint16_t angle = 0;
        uint16_t range = 0;
    }stream;


public:

    AP_Tmxk_LIDARScanner();

    void init();

    void update();

    void doProcess();

    enum Data_Error_Type{
        No_Error_Type    =  0,
        Serial_Miss_Data =  1,  //miss some data byte maybe angle or range in one stream loop
        Serial_Miss_End  =  2,
        Range_Error_Read =  3,
    };

    bool getObstacle() const{
        return scan.isObstacle;
    }

    int16_t getPilotCommand() const{
        return m_pilotCommand;
    }

    int angleConver(int angle);


};


