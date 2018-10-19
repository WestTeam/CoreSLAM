// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_CORESLAM_HPP_
#define WESTBOT_CORESLAM_CORESLAM_HPP_

#include "Map.hpp"
#include "Parameters.hpp"
#include "Position.hpp"
#include "Randomizer.hpp"
#include "Scan.hpp"
#include "SensorData.hpp"
#include "State.hpp"

namespace WestBot {
namespace CoreSLAM {

// ========================================================================== //
// RANDOM: Stochastic part
double ts_random_normal_fix( Randomizer* d );

double ts_random_normal( Randomizer* d, double m, double s );

void ts_random_init( Randomizer* d, unsigned long jsrseed );

double ts_random( Randomizer* d);

long ts_random_int( Randomizer* d, long min, long max );

Position ts_monte_carlo_search(
    Randomizer* randomizer,
    Scan* scan,
    Map* map,
    Position* start_pos,
    double sigma_xy,
    double sigma_theta,
    int stop,
    int* bestdist );

// ========================================================================== //
// Extensions
double ts_distance( Position* pos1, Position* pos2 );

void ts_save_map_pgm(
    Map* map,
    Map* overlay,
    char* filename,
    int width,
    int height );

void ts_draw_scan( Scan* scan, Map* map, Position* pos );

void ts_draw_scan_RGB(
    Scan* scan,
    Map* map,
    Position* pos,
    unsigned char* pixmap,
    int scale,
    int reversey );

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

void ts_build_scan(
    SensorData* sd,
    Scan* scan,
    State* state,
    int span );

void ts_iterative_map_building( SensorData* sd, State* state );

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
