// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_CORESLAM_HPP_
#define WESTBOT_CORESLAM_CORESLAM_HPP_

#include "Defines.hpp"

// ========================================================================== //
// Core
void ts_map_init( ts_map_t* map );

int ts_distance_scan_to_map(
    ts_scan_t* scan,
    ts_map_t* map,
    ts_position_t* pos );

void ts_map_update(
    ts_scan_t* scan,
    ts_map_t* map,
    ts_position_t* position,
    int quality,
    int hole_width );

// ========================================================================== //
// Stochastic part
double ts_random_normal_fix( ts_randomizer_t* d );

double ts_random_normal(ts_randomizer_t* d, double m, double s );

void ts_random_init(ts_randomizer_t* d, unsigned long jsrseed );

double ts_random(ts_randomizer_t *d);

long ts_random_int( ts_randomizer_t* d, long min, long max );

ts_position_t ts_monte_carlo_search(
    ts_randomizer_t* randomizer,
    ts_scan_t* scan,
    ts_map_t* map,
    ts_position_t* start_pos,
    double sigma_xy,
    double sigma_theta,
    int stop,
    int* bestdist );

// ========================================================================== //
// Extensions
double ts_distance( ts_position_t* pos1, ts_position_t* pos2 );

void ts_save_map_pgm(
    ts_map_t* map,
    ts_map_t* overlay,
    char* filename,
    int width,
    int height );

void ts_draw_scan( ts_scan_t* scan, ts_map_t* map, ts_position_t* pos );

void ts_draw_scan_RGB(
    ts_scan_t* scan,
    ts_map_t* map,
    ts_position_t* pos,
    unsigned char* pixmap,
    int scale,
    int reversey );

void ts_state_init(
    ts_state_t* state,
    ts_map_t* map,
    ts_robot_parameters_t* params,
    ts_laser_parameters_t* laser_params,
    ts_position_t* position,
    double sigma_xy,
    double sigma_theta,
    int hole_width,
    int direction );

void ts_build_scan(
    ts_sensor_data_t* sd,
    ts_scan_t* scan,
    ts_state_t* state,
    int span );

void ts_iterative_map_building( ts_sensor_data_t* sd, ts_state_t* state );

// ========================================================================== //
// Loop closing
ts_position_t ts_close_loop_position(
    ts_state_t* state,
    ts_sensor_data_t* sensor_data,
    ts_map_t* loop_close_map,
    ts_position_t* start_position,
    int* q );

void ts_close_loop_trajectory(
    ts_sensor_data_t* sensor_data,
    int maxscans,
    ts_position_t* startpos,
    ts_position_t* close_loop_position );

#endif // WESTBOT_CORESLAM_CORESLAM_HPP_
