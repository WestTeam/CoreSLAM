// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_CORESLAM_HPP_
#define WESTBOT_CORESLAM_CORESLAM_HPP_

#include "Map.hpp"
#include "Parameters.hpp"
#include "Position.hpp"
#include "Scan.hpp"
#include "SensorData.hpp"
#include "State.hpp"

namespace WestBot {
namespace CoreSLAM {
// ========================================================================== //
// Extensions
void ts_save_map_pgm(
    Map* map,
    Map* overlay,
    char* filename,
    int width,
    int height );

// ========================================================================== //
void ts_state_init(
    State* state,
    Map* map,
    RobotParameters* params,
    LaserParameters* laser_params,
    Position* position,
    double sigma_xy,
    double sigma_theta,
    int hole_width,
    int direction );

void ts_iterative_map_building( SensorData* sd, State* state );

void ts_build_scan( SensorData* sd, Scan* scan, State* state, int span );

// ========================================================================== //
// Loop closing
Position ts_close_loop_position(
    State* state,
    SensorData* sensor_data,
    Map* loop_close_map,
    Position* start_position,
    int* q );

void ts_close_loop_trajectory(
    SensorData* sensor_data,
    int maxscans,
    Position* startpos,
    Position* close_loop_position );
}
}

#endif // WESTBOT_CORESLAM_CORESLAM_HPP_
