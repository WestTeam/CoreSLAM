// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_DEFINES_HPP_
#define WESTBOT_CORESLAM_DEFINES_HPP_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TS_SCAN_SIZE 8192
#define TS_MAP_SIZE 2048
#define TS_MAP_SCALE 0.1
#define TS_DISTANCE_NO_DETECTION 4000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0

#define TS_DIRECTION_FORWARD   0
#define TS_DIRECTION_BACKWARD  1
#define TS_FINAL_MAP 2

// ========================================================================== //
// Core
typedef unsigned short ts_map_pixel_t;

typedef struct {
    ts_map_pixel_t map[ TS_MAP_SIZE * TS_MAP_SIZE ];
} ts_map_t;

typedef struct {
    double x[ TS_SCAN_SIZE ];
    double y[ TS_SCAN_SIZE ];
    int value[ TS_SCAN_SIZE ];
    int nb_points;
} ts_scan_t;

typedef struct {
    double x;     // in mm
    double y;     // in mm
    double theta; // in degrees
} ts_position_t;

typedef struct {
    unsigned int timestamp;
    int q1;                      // Odometry information
    int q2;                      // Odometry information
    double v;                    // Used to correct the scans according to the speed of the robot
    double psidot;               // Used to correct the scans according to the speed of the robot
    ts_position_t position[ 3 ]; // 0 : forward - 1 : backward - 2 : final / closed loop
    int d[ TS_SCAN_SIZE ];
} ts_sensor_data_t;

// ========================================================================== //
// Stochastic part
typedef struct {
    unsigned long jz;
    unsigned long jsr;
    long hz;
    unsigned long iz;
    unsigned long kn[ 128 ];
    double wnt[ 128 ];
    double wn[ 128 ];
    double fn[ 128 ];
} ts_randomizer_t;

// ========================================================================== //
// Extensions
typedef struct {
    double r;	    // length wheels' radius
    double R;	    // half the wheels' axis length
    int inc;	    // wheels' counters increments per turn
    double ratio;   // ratio between left and right wheel
} ts_robot_parameters_t;

typedef struct {
    double offset;  // position of the laser wrt center of rotation
    int scan_size;  // number of points per scan
    int angle_min;  // start angle for scan
    int angle_max;  // end angle for scan
    int detection_margin; // first scan element to consider
    double distance_no_detection; // default value when the laser returns 0
} ts_laser_parameters_t;

typedef struct {
    ts_randomizer_t randomizer;
    ts_map_t *map;
    ts_robot_parameters_t params;
    ts_laser_parameters_t laser_params;
    ts_position_t position;
    int q1, q2;
    unsigned int timestamp;
    double psidot, v;
    double distance;
    int hole_width;
    int direction;
    int done, draw_hole_map;
    ts_scan_t scan;
    double sigma_xy;
    double sigma_theta;
} ts_state_t;

#endif // WESTBOT_CORESLAM_DEFINES_HPP_
