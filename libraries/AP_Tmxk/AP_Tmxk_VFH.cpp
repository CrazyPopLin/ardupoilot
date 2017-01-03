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
 * The smooth function:
 * note：CV:Certain_Value； WV:Weight_Value；
 * WV[i]={CV[i-l+1]+...+(l-2)*CV[i-2]+(l-1)*CV[i-1]+l*CV[i]+(l-1)*CV[i+1]+(l-2)*CV[i+2]+...+CV[i+l-1]}/(2*l+1)
 */

#include "AP_Tmxk_VFH.h"
#include <AP_HAL/AP_HAL.h>
#include <math.h>
#include <../ArduCopter/Copter.h>


#define VFH_THRESHOLD_DEFAULT       10
#define VFH_SMOOTH_FACTOR_DEFAULT   5
#define VFH_SCALE_FACTOR_DEFAULT    10

extern const AP_HAL::HAL& hal;

/*const AP_Param::GroupInfo AP_Tmxk_VFH::var_info[] = {
        // @Param: THRESHOLD
        // @DisplayName: threshold
        // @Description:After weight the successive sectors, A threshold will determine which sector belongs to safe valleys
        // @Range: 10-20
        // @Units: scalar variable
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("THRESHOLD",    0, AP_Tmxk_VFH, configuration.threshold, VFH_THRESHOLD_DEFAULT),
        // @Param: SMOOTH_FACTOR
        // @DisplayName: smooth_factor
        // @Description: The smooth factor make successive sectors to take weight
        // @Range: 5-20
        // @Units: scalar variable
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("SMOOTH_FACTOR",    1, AP_Tmxk_VFH, configuration.smooth_factor, VFH_SMOOTH_FACTOR_DEFAULT),
        // @Param: SCALE_FACTOR
        // @DisplayName: scale_factor
        // @Description: Somehow this will used in futture
        // @Range: 10-20
        // @Units: scalar variable
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("SCALE_FACTOR",    2, AP_Tmxk_VFH, configuration.scale_factor, VFH_SCALE_FACTOR_DEFAULT),


        AP_GROUPEND
};*/

AP_Tmxk_VFH::AP_Tmxk_VFH(class AP_Tmxk_LIDARScanner& lidarscanner, const AP_Mission& mission, const AP_InertialNav_NavEKF& inertial_nav,const AP_AHRS_NavEKF& ahrs) :
    l(5),
    threshold(5),
    safe_space(0.7),
    detected_range(2.5),
    theta(degrees(acos(1-sq(safe_space)/(2*sq(detected_range))))),
    //valley_max(round_float(180.0/theta)),
    valley_count(0),
    _lidarscanner(lidarscanner),
    _mission(mission),
    _inertial_nav(inertial_nav),
    _ahrs(ahrs),
    Move(No_Move)
{
    //AP_Param::setup_object_defaults(this, var_info);
}

void AP_Tmxk_VFH::init()
{

}

void AP_Tmxk_VFH::update()
{
    generateVally();
    //uint32_t t1 =AP_HAL::micros();

   /* if(_lidarscanner.LidarScannerAvailable() && _lidarscanner.ScanAvailable()){            //new scan data is available, new lidar data is available

        if (_lidarscanner.getObstacle(sector.Certain_Value))            //get Certain Value (estimation data of obstacle) into sector from lidarscanner
        {
            generateVally();                                            //convert Certain Value to  WeightValue, generate Safe Valley
        }

    }*/
    /*if(generateVally())
    {
        Move = Horizontal_Move;
        //selectDirection();
    }
    else{
        Move = Climb_Up;
    }*/
    //uint32_t t2 = AP_HAL::micros();
    //hal.console->printf("t:%d\n",t2-t1);


    /*if(hal.rcout->read(11) <= 1400) {
        Vector3f v(0,0,0);
        copter.guided_set_velocity(v);
    }
    else if(hal.rcout->read(11) > 1400 ) {
        Vector3f v(0,0,-1);
        copter.guided_set_velocity(v);
    }*/

   /* //get the heading mission wayoint location and the current location
    Location loc_waypoint, loc_now;
    _mission.get_current_nav_cmd_location(loc_waypoint);//test1,check what's return before and after mission uplaod
    _inertial_nav.get_location(loc_now);

    //get vector position from EKF origin in NEU
    Location_Class Loc_Waypoint(loc_waypoint);
    Location_Class Loc_Now(loc_now);
    Vector3f vec_waypoint, vec_now;
    Loc_Waypoint.get_vector_from_origin_NEU(vec_waypoint);//alt is cm
    Loc_Now.get_vector_from_origin_NEU(vec_now);//test2: check both vec is correct

    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();*/

}



/* get smooth of weight value; select continuous sectors whose weight value less than threshold as safe valley;
 * @Param: Weight_Value
 *
 * @return:
 *  1:true: if safe valley existed; false: not existed;
 *  2:safe valley
 *
 */
bool AP_Tmxk_VFH::generateVally()
{
    valley v;
    Valley.clear();
    direction.clear();

    for(int i=0; i<=180; i++){
        bool isSafe = smoothFunction(i,sector.Weight_Value[i] );
        //hal.console->printf("%d,%d\n",i,sector.Weight_Value[i]);
        /***** group 180 sectors into safe valleys    *****/
        if(!inside_valley && isSafe){
            inside_valley = true;
            v.beg_bound = i+1;

        }
        else if (inside_valley && !isSafe){
            //valley_count++;
            inside_valley = false;
            v.angle = i-1 - v.beg_bound;

            if(v.angle >= theta)
                Valley.push_back(v);//remove un-passable safe Valley
        }
    }

    if (Valley.empty())
        return false;
    else
        return true;
}

/* The smooth function, please see above for the detail theory;
 * @Param: index i, the smoothing sector
 * @Param: reference of weight sector which store value after smoothed
 *
 * @return:
 *  1:true:  the value of weight sector (after smoothes) is less than threshold; false: the value of weight sector is grater than threshold;
 *  2:value of weight sector
 *
 */
bool AP_Tmxk_VFH::smoothFunction(int i, uint8_t &i_)
{
    double sum =0;
    for(int j = i-l+1, count =1; j <= i+l-1; count++, j++ ){
        if(j >=0 && j <=180){
              uint8_t factor = (count > l) ? 2*l-count : count;
               sum = sum+ sector.Certain_Value[j] * factor;
        }
    }
    i_ =round_float( sum/(2*l+1));//high value represent high possibility of obstacle in this sector which is about 1 degree

    return i_<threshold;
}

int AP_Tmxk_VFH::selectDirection()
{
    for (std::vector<valley>::iterator it = Valley.begin() ; it != Valley.end(); ++it){
        direction.push_back(it->beg_bound + theta/2);
        direction.push_back(it->beg_bound + it->angle - theta/2);
    }
    //choose the direction which closet to target_direction to be assigned

    return 50;

}
