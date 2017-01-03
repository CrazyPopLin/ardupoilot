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
    while(hal.uartE->available()){
        /****Monitor if lidarscanner works for VFH invoking****/
        monitor_time = AP_HAL::micros();
        isScannerWork = true;

         /****Decode from stream loop and get angle and range****/
        int r = hal.uartE->read();

        if (stream.recvInProgress == true){
            if (r != '>') {
                stream.received_Byte[stream.count] = r;
                stream.count++;
                if (stream.count >= RECEIVED_BYTE_MAX) {
                    stream.count = RECEIVED_BYTE_MAX - 1;
                }
            }
            else {
                stream.received_Byte[stream.count] = '\0'; // terminate the string
                stream.recvInProgress = false;

                // check if data is miss in this stream loop
                if(stream.count == RECEIVED_BYTE_LEN)
                {
                    stream.error = No_Error_Type;
                    //convert highbyte and low byte into 16byte int
                    stream.angle = (stream.received_Byte[0] << 8 ) | (stream.received_Byte[1] & 0xff);
                    stream.range = (stream.received_Byte[2] << 8 ) | (stream.received_Byte[3] & 0xff);
                }
                else if(stream.count < RECEIVED_BYTE_LEN){
                    stream.error = Serial_Miss_Data;
                    //store data as previous one as angle interval is about 1 degree
                    stream.angle = scan.AngleArr[scan.StreamCount-1] +1;
                    stream.range = scan.RangeArr[scan.StreamCount-1];
                }
                else{
                    stream.error = Serial_Miss_End;
                    //convert first 4 byte into angle and range, then left the miss parts??
                    stream.angle = (stream.received_Byte[0] << 8 ) | (stream.received_Byte[1] & 0xff);
                    stream.range = (stream.received_Byte[2] << 8 ) | (stream.received_Byte[3] & 0xff);
                }

                //if zero position is detected then process the each scan data and clear the old one
                if (stream.angle ==0) {
                    scan.available = false;
                    checkObstacle();
                    scan.available = true;      //means new scan data is available
                }

                //store each stream data to scan period
                scan.AngleArr[scan.StreamCount] = stream.angle;
                scan.RangeArr[scan.StreamCount] = stream.range;
                //hal.console->printf("%d,%d,%d\n",stream.angle,stream.range,scan.StreamCount);//LS RUN 400HZ, the output which changes at every scan

                scan.StreamCount++;
                if(scan.StreamCount>=LIDAR_STREAM){
                    scan.StreamCount=LIDAR_STREAM-1;
                }
                memset(stream.received_Byte, 0, RECEIVED_BYTE_MAX);
                stream.count = 0;
            }
        }
        else if ( r == '<' ){
            stream.recvInProgress = true;
        }
    }

    if (AP_HAL::micros()- monitor_time > 1000000)
        isScannerWork = false;
}

//determine if some detected range of obstacle are dangerous
void AP_Tmxk_LIDARScanner::checkObstacle()
{
    scan.ErrorCount = 0;
    memset(scan.isObstacle, 0, LIDAR_STREAM/2+1);
    bool flag =false;

    //check if safe or not for each circle
    int tranfered_angle;
    for(int i =0; i < scan.StreamCount; i++ ){

        tranfered_angle = angleTranfer(scan.AngleArr[i]);//return 0-180 degree if angle is 180-360; else return -1;some angle missed because lidarscanner not stable

        if      ( tranfered_angle != -1 && scan.RangeArr[i] <  UPPER && scan.RangeArr[i] >  LOWER ){//angle from 0-180, found obstacle from LOWER to UPPER
            scan.isObstacle[tranfered_angle] = true;
            flag = true;
        }

       //if(tranfered_angle != -1)
           // hal.console->printf("%d,%d,%d\n",tranfered_angle,scan.RangeArr[i],flag);

        if (scan.RangeArr[i] <= 1 || scan.RangeArr[i] >= 4000 ){
            scan.ErrorCount++;//record amount of error data in each circle
        }
    }

   //flag indicated ture: if any angle found obstacle
    if(flag){
        buff_filter = (buff_filter+1 >= UPDATE_OBSTACLE_MAX) ? UPDATE_OBSTACLE_MAX : buff_filter+1;// value always not greater than UPDATE_OBSTACLE_MAX
    }
    else{
        buff_filter= (buff_filter-1 >= UPDATE_OBSTACLE_MIN) ? buff_filter-1 : UPDATE_OBSTACLE_MIN;//value always not less than UPDATE_OBSTACLE_MIN
    }

    if( buff_filter >= 2 ){
        isSafe = false;
    }
    else{
        isSafe = true;
    }
    //hal.console->printf("\n%d\n",buff_filter);

    //clear scan struct data
    memset(scan.AngleArr, 0, LIDAR_STREAM);
    memset(scan.RangeArr, 0, LIDAR_STREAM);
    scan.StreamCount = 0;
}

/* The process update rare lidarscanner data to estimation of obstacle data, which will update 1 if found obstacle in this sector (each 1 degree) ;
 * @Param: reference of sector array (defined in AP_Tmxk_VFH.h)
 *
 * @return:
 *  1:true: if finished this process for future determined.
 *  2:estimation value of obstacle
 *
 */
bool AP_Tmxk_LIDARScanner::getObstacle(uint8_t (&array)[LIDAR_STREAM/2+1])//uint8_t (&array)[LIDAR_STREAM/2+1]
{
    for(int i=0; i<=LIDAR_STREAM/2; i++){
        //hal.console->printf("%d,%d\n",i,scan.isObstacle[i]);
        if(scan.isObstacle[i]){
            array[i] = (array[i]+1 >= UPDATE_OBSTACLE_MAX) ? UPDATE_OBSTACLE_MAX : array[i]+1;// value always not greater than UPDATE_OBSTACLE_MAX
        }
        else{
            array[i] = (array[i]-1 >= UPDATE_OBSTACLE_MIN) ? array[i]-1 : UPDATE_OBSTACLE_MIN;//value always not less than UPDATE_OBSTACLE_MIN
        }
    }
    return true;
}



