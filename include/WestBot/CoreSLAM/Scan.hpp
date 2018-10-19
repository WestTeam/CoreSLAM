// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_SCAN_HPP_
#define WESTBOT_CORESLAM_SCAN_HPP_

#include "Defines.hpp"

namespace WestBot {
namespace CoreSLAM {

class Map;
class Position;

class Scan
{
public:
    Scan() = default;

    void ts_draw_scan( Map* map, Position* pos );

    void ts_draw_scan_RGB(
        Map* map,
        Position* pos,
        unsigned char* pixmap,
        int scale,
        int reversey );

    double x[ TS_SCAN_SIZE ];
    double y[ TS_SCAN_SIZE ];
    int value[ TS_SCAN_SIZE ];
    int nb_points;
};

}
}

#endif // WESTBOT_CORESLAM_SCAN_HPP_
