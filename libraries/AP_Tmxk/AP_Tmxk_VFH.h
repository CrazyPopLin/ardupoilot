
#pragma once


#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <stdio.h>
#include <vector>

#include "AP_Tmxk_LIDARScanner.h"



#define PI 3.1415926

class AP_Tmxk_VFH {

private:

    uint8_t l;              //length factor to weight Certain_Value
    uint8_t threshold;      //it defined by experiments, print all the weight value to determined. the unit is uint8_t, it's enough
    float   safe_space;     //unit: m; safe space for drone go through, normal equal to D_vehicle(31cm)+ D_propeller(17cm) +Offset (offset usually select as D_propeller )
    float   detected_range; //equal to UPPER in AP_Tmxk_LIDARScanner
    float   theta          ;//min angle of space to go through 16.0957

    uint8_t valley_count = 0 ;
    bool inside_valley   = false;

    int flag=0;

    struct{
        bool Obstacle_Estm[181]    = {};
        uint8_t Certain_Value[181] = {};
        uint8_t Weight_Value[181]  = {};
    }sector;

    struct valley{
        uint16_t beg_bound = 0;  //begin bound of valley which is about small angle
        uint16_t angle     = 0;  //the entire angle of this valley
    };

    std::vector<valley> Valley;

    std::vector<uint8_t> direction;

    int target_direction =0;

    class AP_Tmxk_LIDARScanner& _lidarscanner;

    const AP_Mission& _mission;

    const AP_InertialNav_NavEKF& _inertial_nav;

    const AP_AHRS_NavEKF& _ahrs;

public:

    // Constructor
    AP_Tmxk_VFH(class AP_Tmxk_LIDARScanner &_lidarscanner, const AP_Mission& _mission, const AP_InertialNav_NavEKF& _inertial_nav, const AP_AHRS_NavEKF& _ahrs);

    void init();

    void update();

    enum Motion_Type {
        No_Move         = 0,
        Climb_Up        = 1,
        Horizontal_Move = 2,
    }Move;

    bool generateVally();           // thing about how to init function in struct

    bool smoothFunction(int i, uint8_t &i_);

    int selectDirection();

    uint8_t round_float(float number)
    {
        return (number > 0.0) ? (number + 0.5) : (number - 0.5);
    }


    // for holding parameters
    //static const struct AP_Param::GroupInfo var_info[];

protected:
    //configuration to adjust vfh performance
    struct{
        AP_Int8 threshold;
        AP_Int8 smooth_factor;
        AP_Int8 scale_factor;
    }configuration;
};
