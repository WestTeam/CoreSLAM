// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_POSITION_HPP_
#define WESTBOT_CORESLAM_POSITION_HPP_

namespace WestBot {
namespace CoreSLAM {

class Position
{
public:
    Position() = default;

    static double ts_distance( Position* pos1, Position* pos2 );

    double x;     // in mm
    double y;     // in mm
    double theta; // in degrees
};

}
}

#endif // WESTBOT_CORESLAM_POSITION_HPP_
