// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_PARAMETERS_HPP_
#define WESTBOT_CORESLAM_PARAMETERS_HPP_

namespace WestBot {
namespace CoreSLAM {

struct RobotParameters
{
    double r;	    // length wheels' radius
    double R;	    // half the wheels' axis length
    int inc;	    // wheels' counters increments per turn
    double ratio;   // ratio between left and right wheel
};

struct LaserParameters
{
    double offset;  // position of the laser wrt center of rotation
    int scan_size;  // number of points per scan
    int angle_min;  // start angle for scan
    int angle_max;  // end angle for scan
    int detection_margin; // first scan element to consider
    double distance_no_detection; // default value when the laser returns 0
};

}
}

#endif // WESTBOT_CORESLAM_PARAMETERS_HPP_
