/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */




#include "AP_Tmxk_LIDARScanner.h"

#include <../ArduCopter/Copter.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;



AP_Tmxk_LIDARScanner::AP_Tmxk_LIDARScanner(void)
{
    m_pilotCommand = false;
}

void AP_Tmxk_LIDARScanner::init()
{

}

void AP_Tmxk_LIDARScanner::update()
{
    //uint32_t t1 =AP_HAL::micros();

    while(hal.uartE->available()){
         //Decode from stream loop and get angle and range
        int r = hal.uartE->read();

        if (stream.recvInProgress == true){
            if (r != '>') {
                stream.received_Byte[stream.count] = r;
                stream.count++;
                if (stream.count >= RECEIVED_DATA_MAX) {
                    stream.count = RECEIVED_DATA_MAX - 1;
                }
            }
            else {
                stream.received_Byte[stream.count] = '\0'; // terminate the string
                stream.recvInProgress = false;

                // check if data is miss in this stream loop
                if(stream.count == RECEIVED_DATA_LEN)
                {
                    stream.error = No_Error_Type;
                    //convert highbyte and low byte into 16byte int
                    stream.angle = (stream.received_Byte[0] << 8 ) | (stream.received_Byte[1] & 0xff);
                    stream.range = (stream.received_Byte[2] << 8 ) | (stream.received_Byte[3] & 0xff);
                }
                else if(stream.count < RECEIVED_DATA_LEN){
                    stream.error = Serial_Miss_Data;
                    //store data as previous one as angle interval is about 1 degree
                    stream.angle = scan.AngleArr[scan.StreamCount-1] +1;
                    stream.range = scan.RangeArr[scan.StreamCount-1];
                }
                else{
                    stream.error = Serial_Miss_End;
                }
                //if zero position is detected then process the each scan data and clear the old one
                if (stream.angle ==0) {
                   // hal.console->printf("\ncount=%d\n",  scan.StreamCount);
                    doProcess();
                }
                //store each stream data to scan period
                scan.AngleArr[scan.StreamCount] = stream.angle;
                scan.RangeArr[scan.StreamCount] = stream.range;

                scan.StreamCount++;
                if(scan.StreamCount>=LIDAR_STREAM){
                    scan.StreamCount=LIDAR_STREAM-1;
                }
                memset(stream.received_Byte, 0, RECEIVED_DATA_MAX);
                stream.count = 0;
            }
        }
        else if ( r == '<' ){
            stream.recvInProgress = true;
        }
    }

    if(hal.rcout->read(11) <= 1400) {
        Vector3f v = {0,0,0};
        copter.guided_set_velocity(v);
    }
    else if(hal.rcout->read(11) > 1400 ) {
        Vector3f v = {0,0,-1};
        copter.guided_set_velocity(v);
    }


    //uint32_t t2 = AP_HAL::micros();
    //hal.console->printf("t:%d\n",t2-t1);
}

//determine if some detected range of obstacle are dangerous
void AP_Tmxk_LIDARScanner::doProcess()
{

    int angle;
    scan.ErrorCount = 0;

    memset(scan.isObstacle, 0, LIDAR_STREAM/2);
    scan.available = false;

    //check if safe or not for each circle
   for(int i =0; i < scan.StreamCount; i++ ){

        angle = angleConver(scan.AngleArr[i]);//return 0-180 degree if angle is 180-360; else return -1

        if      ( angle != -1 && scan.RangeArr[i] <  UPPER && scan.RangeArr[i] >  LOWER ){//angle from 0-180, found obstacle from LOWER to UPPER
            scan.isObstacle[angle] = true;
        }
        else if ( angle != -1 && scan.RangeArr[i] <= LOWER && scan.RangeArr[i] >= UPPER ) {
            scan.isObstacle[angle] = false;
        }

        if (scan.RangeArr[i] <= 1 || scan.RangeArr[i] >= 4000 ){
            scan.ErrorCount++;//record amount of error data in each circle
        }
    }
    scan.available = true;

    //clear scan struct data
    memset(scan.AngleArr, 0, LIDAR_STREAM);
    memset(scan.RangeArr, 0, LIDAR_STREAM);
    scan.StreamCount = 0;

}

int AP_Tmxk_LIDARScanner::angleConver(int _angle)
{
    return (_angle -180 >= 0 ) ? _angle-180 : -1;
}



