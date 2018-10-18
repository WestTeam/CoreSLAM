// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_SENSORDATA_HPP_
#define WESTBOT_CORESLAM_SENSORDATA_HPP_

#include "Defines.hpp"
#include "Position.hpp"

namespace WestBot {
namespace CoreSLAM {

class SensorData
{
public:
    SensorData() = default;

    unsigned int timestamp;
    int q1;                 // Odometry information
    int q2;                 // Odometry information
    double v;               // Used to correct the scans according to the speed of the robot
    double psidot;          // Used to correct the scans according to the speed of the robot
    Position position[ 3 ]; // 0 : forward - 1 : backward - 2 : final / closed loop
    int d[ TS_SCAN_SIZE ];
};

}
}

#endif // WESTBOT_CORESLAM_SENSORDATA_HPP_
