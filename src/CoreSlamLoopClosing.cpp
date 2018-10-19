// Copyright (c) 2018 All Rights Reserved WestBot

#include <math.h>

#include <WestBot/CoreSLAM/CoreSlam.hpp>
#include <WestBot/CoreSLAM/Randomizer.hpp>
#include <WestBot/CoreSLAM/Scan.hpp>
#include <WestBot/CoreSLAM/State.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

Position WestBot::CoreSLAM::ts_close_loop_position(
    State* state,
    SensorData* sensor_data,
    Map* loop_close_map,
    Position* start_position,
    int* q )
{
    Scan scan;
    int quality;
    Position lc_position;

    state->ts_build_scan(sensor_data, &scan, 1);
    lc_position = state->randomizer.ts_monte_carlo_search(&scan, loop_close_map, start_position, 600, 20, 100000, &quality);
    if (q) *q = quality;
    return lc_position;
}

void WestBot::CoreSLAM::ts_close_loop_trajectory(
    SensorData* sensor_data,
    int maxscans,
    Position* startpos,
    Position* close_loop_position )
{
    int i, j;
    double weight, theta[2];
    Position* final_pos;
    for (i = 0; i != maxscans; i++) {
        weight = i / ((double)(maxscans - 1));
        final_pos = &sensor_data[i].position[TS_FINAL_MAP];
        final_pos->x = (1 - weight) * sensor_data[i].position[TS_DIRECTION_FORWARD].x + weight * sensor_data[i].position[TS_DIRECTION_BACKWARD].x;
        final_pos->y = (1 - weight) * sensor_data[i].position[TS_DIRECTION_FORWARD].y + weight * sensor_data[i].position[TS_DIRECTION_BACKWARD].y;
        // Make sure both angles are between 0 and 360
        for (j = 0; j != 2; j++) {
            theta[j] = sensor_data[i].position[j].theta;
            while (theta[j] < 0) theta[j] += 360;
            while (theta[j] >= 360) theta[j] -= 360;
        }
        if (fabs(theta[0] - theta[1]) >= 180)
            if (theta[0] > theta[1])
                theta[0] -= 360;
            else
                theta[1] -= 360;
        final_pos->theta = (1 - weight) * theta[0] + weight * theta[1];
    }
}
