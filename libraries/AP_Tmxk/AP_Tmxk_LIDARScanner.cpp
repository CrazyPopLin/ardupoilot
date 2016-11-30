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

/*
  AP_Tmxk_LIDARSCANNER take care of coming serial data from ardunio nano type LIDARScanner data with serial bytes.
  This Library will be called at 400hz in ArduCopter.cpp so that no delay called in flight control loop

  By Linzhao
  System Engineer in Skywalker Innovation Technology
 */

#include "AP_Tmxk_LIDARScanner.h"

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;


/*const AP_Param::GroupInfo AP_Tmxk_LIDARScanner::var_info[] PROGMEM = {
        // @Param: STREAM
        // @DisplayName: Set up lidar stream parameter
        // @Description: Set up how many stream (readings) in each cycle
        // @Range: 1 360
        // @Units: Radians
        // @Increment: 0.01
        // @User: Standard
        AP_GROUPINFO("STREAM", 1, AP_Tmxk_LIDARScanner, lidar_stream_parameter, lidar_stream_param_default),

        AP_GROUPEND
};*/

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
                else{
                    stream.error = Serial_Miss_Data;
                    //store data as previous one as angle interval is about 1 degree
                    stream.angle = scan.AngleArr[scan.StreamCount-1] +1;
                    stream.range = scan.RangeArr[scan.StreamCount-1];
                }

                if (stream.angle ==0) {
                    doProcess();//if zero position is detected then process the each scan data and clear the old one
                    //clear
                    memset(scan.AngleArr, 0, LIDAR_STREAM);
                    memset(scan.RangeArr, 0, LIDAR_STREAM);
                    scan.StreamCount = 0;
                }
                //store each stream data to scan period
                scan.AngleArr[scan.StreamCount] = stream.angle;
                scan.RangeArr[scan.StreamCount] = stream.range;
                //hal.console->printf("%d,%d,%d\n", stream.angle,stream.range, stream.count);

                stream.count = 0;
                memset(stream.received_Byte, 0, RECEIVED_DATA_MAX);
                scan.StreamCount++;
            }
        }
        else if ( r == '<' ){
            stream.recvInProgress = true;
        }
    }

    //uint32_t t2 = AP_HAL::micros();
    //hal.console->printf("t:%d\n",t2-t1);
}

//determine if some detected range of obstacle are dangerous
void AP_Tmxk_LIDARScanner::doProcess()
{
    int closest = SAFE_RANGE;
    int flag = -1;// indicate safe
    scan.ErrorCount = 0;

    //check if safe or not for each circle
    for(int i =0; i < scan.StreamCount; i++ ){
        if ( (scan.AngleArr[i] >= 180 && scan.AngleArr[i] <= 360) && (scan.RangeArr[i] < SAFE_RANGE && scan.RangeArr[i] > ERROR_RANGE ) && scan.RangeArr[i] < closest){
            closest = scan.RangeArr[i];
            flag = i;
        }
        if (scan.RangeArr[i] <= 1 || scan.RangeArr[i] >= 4000 ){
            scan.ErrorCount++;//record amount of error data in each circle
        }
    }

    //filter error data in case make copter oscillation
    if (flag != -1){
         scan.ClosedAngle = scan.AngleArr[flag];
         scan.ClosedRange = scan.RangeArr[flag];

         scan.buff_filter = (scan.buff_filter+1 >= scan.Buff_Filter_Max) ? scan.Buff_Filter_Max : scan.buff_filter+1;
    }
    else{
         scan.ClosedAngle = -1;
         scan.ClosedRange = -1;

         scan.buff_filter = (scan.buff_filter-1 >= scan.Buff_Filter_Min) ? scan.buff_filter-1 : scan.Buff_Filter_Min;
    }
    //hal.console->printf("ang=%d, rng=%d, filter=%d\n",scan.ClosedAngle,scan.ClosedRange, scan.buff_filter);

    //send the closest angle in each circle to gcs for monitoring
    //hal.rcin->set_override(5,scan.ClosedAngle);

    //determine final safety depend on filtered value
    if( scan.buff_filter >= (scan.Buff_Filter_Max+1)/2 ){
        scan.isSafe = false;
    }
    else{
        scan.isSafe = true;
    }

}


