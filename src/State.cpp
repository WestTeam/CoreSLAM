// Copyright (c) 2018 All Rights Reserved WestBot

#include <stdlib.h>
#include <math.h>

#include <WestBot/CoreSLAM/SensorData.hpp>
#include <WestBot/CoreSLAM/State.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

State::State()
{
}

void State::ts_state_init(
    Map* map,
    RobotParameters* params,
    LaserParameters* laser_params,
    Position* position,
    double sigma_xy,
    double sigma_theta,
    int hole_width,
    int direction )
{
    this->randomizer.ts_random_init( 0xdead );
    this->map = map;
    this->params = *params;
    this->laser_params = *laser_params;
    this->position = *position;
    this->timestamp = 0; // Indicating start
    this->distance = 0;
    this->q1 = q2 = 0;
    this->psidot = v = 0;
    this->direction = direction;
    this->done = 0;
    this->draw_hole_map = 0;
    this->sigma_xy = sigma_xy;
    this->sigma_theta = sigma_theta;
    this->hole_width = hole_width;
}

void State::ts_build_scan(
    SensorData* sd,
    Scan* scan,
    int span )
{
    int i, j;
    double angle_rad, angle_deg;

    scan->nb_points = 0;
    // Span the laser scans to better cosd->ver the space
    for (i = 0; i < laser_params.scanSize(); i++) {
        for (j = 0; j != span; j++) {
            angle_deg = laser_params.angleMin() + ((double)(i * span + j)) * (laser_params.angleMax() - laser_params.angleMin()) / (laser_params.scanSize() * span - 1);
            angle_deg += sd->psidot / 3600.0 * ((double)(i * span + j)) * (laser_params.angleMax() - laser_params.angleMin()) / (laser_params.scanSize() * span - 1);

            angle_rad = angle_deg * M_PI / 180;
            if (i > laser_params.detectionMargin() && i < laser_params.scanSize() - laser_params.detectionMargin()) {
                if (sd->d[i] == 0) {
                    scan->x[scan->nb_points] = laser_params.distanceNoDetection() * cos(angle_rad);
                    scan->x[scan->nb_points] -= sd->v * 1000 * ((double)(i * span + j)) * (laser_params.angleMax() - laser_params.angleMin()) / (laser_params.scanSize() * span - 1) / 3600.0;
                    scan->y[scan->nb_points] = laser_params.distanceNoDetection() * sin(angle_rad);
                    scan->value[scan->nb_points] = TS_NO_OBSTACLE;
                    scan->nb_points++;
                }
                if (sd->d[i] > hole_width / 2) {
                    scan->x[scan->nb_points] = sd->d[i] * cos(angle_rad);
                    scan->x[scan->nb_points] -= sd->v * 1000 * ((double)(i * span + j)) * (laser_params.angleMax() - laser_params.angleMin()) / (laser_params.scanSize() * span - 1) / 3600.0;
                    scan->y[scan->nb_points] = sd->d[i] * sin(angle_rad);
                    scan->value[scan->nb_points] = TS_OBSTACLE;
                    scan->nb_points++;
                }
            }
        }
    }
}

void State::ts_iterative_map_building( SensorData* sd )
{
    double psidot, v, d;
    Scan scan2map;
    double m, thetarad;
    Position position;

    // Manage robot position
    if (timestamp != 0) {
        m = params.wheelRadius() * M_PI / params.inc();
        v = m * (sd->q1 - q1 + (sd->q2 - q2) * params.ratio());
        thetarad = position.theta * M_PI / 180;
        position = this->position;
        position.x += v * 1000 * cos(thetarad);
        position.y += v * 1000 * sin(thetarad);
        psidot = (m * ((sd->q2 - q2) * params.ratio() - sd->q1 + q1) / params.halfWheelAxisLength()) * 180 / M_PI;
        position.theta += psidot;
        v *= 1000000.0 / (sd->timestamp - timestamp);
        psidot *= 1000000.0 / (sd->timestamp - timestamp);
    } else {
        psidot = psidot = 0;
        v = v = 0;
        position = this->position;
        thetarad = position.theta * M_PI / 180;
    }

    // Change to (x,y) scan
    if (direction == TS_DIRECTION_FORWARD) {
        // Prepare speed/yawrate correction of scans (with a delay of 1 clock)
        sd->psidot = psidot;
        sd->v = v;
    }
    ts_build_scan(sd, &scan2map, 3);
    ts_build_scan(sd, &scan, 1);

    // Monte Carlo search
    position.x += laser_params.offset() * cos(thetarad);
    position.y += laser_params.offset() * sin(thetarad);
    sd->position[direction] = position =
        randomizer.ts_monte_carlo_search(&scan, map, &position, sigma_xy, sigma_theta, 1000, NULL);
    sd->position[direction].x -= laser_params.offset() * cos(position.theta * M_PI / 180);
    sd->position[direction].y -= laser_params.offset() * sin(position.theta * M_PI / 180);
    d = sqrt((position.x - sd->position[direction].x) * (position.x - sd->position[direction].x) +
            (position.y - sd->position[direction].y) * (position.y - sd->position[direction].y));
    distance += d;

    // Map update
    map->ts_map_update(&scan2map, &position, 50, hole_width);

    // Prepare next step
    position = sd->position[direction];
    this->psidot = psidot;
    this->v = v;
    q1 = sd->q1;
    q2 = sd->q2;
    timestamp = sd->timestamp;
}
