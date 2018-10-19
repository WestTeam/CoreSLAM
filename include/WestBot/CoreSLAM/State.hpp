// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_STATE_HPP_
#define WESTBOT_CORESLAM_STATE_HPP_

#include "Map.hpp"
#include "Parameters.hpp"
#include "Position.hpp"
#include "Randomizer.hpp"
#include "Scan.hpp"

namespace WestBot {
namespace CoreSLAM {

class SensorData;

class State
{
public:
    State();

    void ts_state_init(
        Map* map,
        RobotParameters* params,
        LaserParameters* laser_params,
        Position* position,
        double sigma_xy,
        double sigma_theta,
        int hole_width,
        int direction );

    void ts_iterative_map_building( SensorData* sd );

    void ts_build_scan( SensorData* sd, Scan* scan, int span );

    Randomizer randomizer;
    Map* map;
    RobotParameters params;
    LaserParameters laser_params;
    Position position;
    int q1;
    int q2;
    unsigned int timestamp;
    double psidot;
    double v;
    double distance;
    int hole_width;
    int direction;
    int done;
    int draw_hole_map;
    Scan scan;
    double sigma_xy;
    double sigma_theta;
};

}
}

#endif // WESTBOT_CORESLAM_STATE_HPP_
