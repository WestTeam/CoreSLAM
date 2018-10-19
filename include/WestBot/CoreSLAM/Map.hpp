// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_MAP_HPP_
#define WESTBOT_CORESLAM_MAP_HPP_

#include "Defines.hpp"

namespace WestBot {
namespace CoreSLAM {

class Scan;
class Position;

typedef unsigned short ts_map_pixel_t;

class Map
{
public:
    Map();

    void ts_map_init();

    int ts_distance_scan_to_map( Scan* scan, Position* pos );

    void ts_map_update(
        Scan* scan,
        Position* position,
        int quality,
        int hole_width );

    ts_map_pixel_t map[ TS_MAP_SIZE * TS_MAP_SIZE ];

private:
    void ts_map_laser_ray(
        int x1,
        int y1,
        int x2,
        int y2,
        int xp,
        int yp,
        int value,
        int alpha );
};

}
}

#endif // WESTBOT_CORESLAM_MAP_HPP_
