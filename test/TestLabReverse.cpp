// Copyright (c) 2018 All Rights Reserved WestBot

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "opencv2/opencv.hpp"

#include <WestBot/CoreSLAM/CoreSlam.hpp>
#include <WestBot/CoreSLAM/Defines.hpp>

#define TEST_FILENAME "test_lab.dat"
#define TEST_SCAN_SIZE 682
#define TEST_MIN_DIST 20
#define TEST_ANGLE_MIN -120
#define TEST_ANGLE_MAX +120
#define TEST_OFFSET_LASER 145
#define TEST_HOLE_WIDTH 600

using namespace WestBot;
using namespace WestBot::CoreSLAM;

Map trajectory;
Map map;

typedef struct {
    double r;	    // length wheels' radius
    double R;	    // half the wheels' axis length
    int inc;	    // wheels' counters increments per turn
    double ratio;   // ratio between left and right wheel
} cart_parameters_t;

typedef struct {
    int timestamp;
    int q1, q2;
    Scan scan;
} ts_sensor_data_t2;

ts_sensor_data_t2 sensor_data[600];

void record_map(
    Map* map,
    Map* overlay,
    char* filename,
    int width,
    int height )
{
    int x, y, xp, yp;
    FILE *output;
    output = fopen(filename, "wt");
    fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
        x = (TS_MAP_SIZE - width) / 2;
	for (xp = 0; xp < width; x++, xp++) {
	    if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0)
                fprintf(output, "0 ");
            else
                fprintf(output, "%d ", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
	}
	fprintf(output, "\n");
    }
    fclose(output);
}

void draw_map(
    Map* map,
    Map* overlay,
    int width,
    int height )
{
    int x, y, xp, yp;
    cv::Mat image(height,width,CV_8UC3);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
      x = (TS_MAP_SIZE - width) / 2;
      for (xp = 0; xp < width; x++, xp++) {
	unsigned char *p = image.data+yp*width*3+xp*3;
	if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0) {
	  p[0] = 0; p[1] = 0; p[2] = 0;
	}
	else if(overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 254) {
	  p[0] = 0; p[1] = 0; p[2] = 255;
	}
	else {
	  //p[0] = 255; p[1] = 0; p[2] = ;
	  p[0] = (map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
	  p[1] = (map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
	  p[2] = (map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8;
	}
      }
    }
    cv::imshow("1",image);
    cv::waitKey(10);
}

void draw_map2( Map* map, Map* overlay, int width, int height )
{
    int x, y, xp, yp;
    cv::Mat image(height,width,CV_8UC3);
    for (yp = 0; yp < height; y++, yp++) {
      for (xp = 0; xp < width; x++, xp++) {
	unsigned char *p = image.data+yp*width*3+xp*3;
	if (overlay->map[ yp * TS_MAP_SIZE + xp] == 0) {
	  p[0] = 0; p[1] = 0; p[2] = 0;
	}
	else if(overlay->map[yp * TS_MAP_SIZE + xp] == 254) {
	  p[0] = 0; p[1] = 0; p[2] = 255;
	}
	else {
	  //p[0] = 255; p[1] = 0; p[2] = ;
	  p[0] = (map->map[yp * TS_MAP_SIZE + xp]) >> 8;
	  p[1] = (map->map[yp * TS_MAP_SIZE + xp]) >> 8;
	  p[2] = (map->map[yp * TS_MAP_SIZE + xp]) >> 8;
	}
      }
    }
    cv::imshow("1",image);
    cv::waitKey(10);
}

void draw_scan( Scan* scan, Map* map, Position* pos )
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x2p = c * scan->x[i] - s * scan->y[i];
            y2p = s * scan->x[i] + c * scan->y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE;
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE)
                map->map[y2 * TS_MAP_SIZE + x2] = 0;
        }
    }
}

Position monte_carlo_move(
    Scan* scan,
    Map* map,
    Position* start_pos,
    int debug )
{
    Position cpp, currentpos, bestpos, lastbestpos;
    int currentdist;
    int bestdist, lastbestdist;
    int counter = 0;

    currentpos = bestpos = lastbestpos = *start_pos;
    currentdist = ts_distance_scan_to_map(scan, map, &currentpos);
    bestdist = lastbestdist = currentdist;

    do {
	currentpos = lastbestpos;
	currentpos.x += 50 * (((double)rand()) / RAND_MAX - 0.5);
	currentpos.y += 50 * (((double)rand()) / RAND_MAX - 0.5);
	currentpos.theta += 50 * (((double)rand()) / RAND_MAX - 0.5);

	currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

	if (currentdist < bestdist) {
	    bestdist = currentdist;
	    bestpos = currentpos;
            if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n", bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
	} else {
	    counter++;
	}
        if (counter > 100) {
            if (bestdist < lastbestdist) {
                lastbestpos = bestpos;
                lastbestdist = bestdist;
                counter = 0;
            }
        }
    } while (counter < 1000);
    return bestpos;
}

int read_sensor_data( char* inputfile, ts_sensor_data_t2* data )
{
    FILE *input;
    int i, j, nb_sensor_data = 0;
    int d[TS_SCAN_SIZE];
    Scan *scan;
    char *str, line[4000];
    double angle_deg, angle_rad;

    input = fopen(inputfile, "rt");
    do {
        // Read the scan
        str = fgets(line, 4000, input);
        if (str == NULL) break;
        str = strtok(str, " ");
        sscanf(str, "%d", &data[nb_sensor_data].timestamp);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &data[nb_sensor_data].q1);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &data[nb_sensor_data].q2);
        data[nb_sensor_data].q2 = -data[nb_sensor_data].q2;
        for (i = 0; i < 10; i++)
            str = strtok(NULL, " ");
        for (i = 0; i < TEST_SCAN_SIZE; i++) {
            if (str) {
                sscanf(str, "%d", &d[i]);
                str = strtok(NULL, " ");
            } else d[i] = 0;
        }

        // Change to (x,y) scan
        scan = &data[nb_sensor_data].scan;
        scan->nb_points = 0;
#define SPAN 3
        // Span the laser scans to better cover the space
        for (i = 0; i < TEST_SCAN_SIZE; i++) {
            for (j = 0; j != SPAN; j++) {
                angle_deg = TEST_ANGLE_MIN + ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);

                // Correction of angle according to odometry
                //angle_deg += psidotodo_old / 3600.0 * ((double)(i * SPAN + j)) * (TEST_ANGLE_MAX - TEST_ANGLE_MIN) / (TEST_SCAN_SIZE * SPAN - 1);

                angle_rad = angle_deg * M_PI / 180;
                if (i > 45 && i < TEST_SCAN_SIZE - 45) {
                    if (d[i] == 0) {
                        scan->x[scan->nb_points] = TS_DISTANCE_NO_DETECTION * cos(angle_rad);
                        scan->y[scan->nb_points] = TS_DISTANCE_NO_DETECTION * sin(angle_rad);
                        scan->value[scan->nb_points] = TS_NO_OBSTACLE;
                        scan->x[scan->nb_points] += TEST_OFFSET_LASER;
                        scan->nb_points++;
                    }
                    if (d[i] > TEST_HOLE_WIDTH / 2) {
                        scan->x[scan->nb_points] = d[i] * cos(angle_rad);
                        scan->y[scan->nb_points] = d[i] * sin(angle_rad);
                        scan->value[scan->nb_points] = TS_OBSTACLE;
                        scan->x[scan->nb_points] += TEST_OFFSET_LASER;
                        scan->nb_points++;
                    }
                }
            }
        }
        nb_sensor_data++;
    } while (1);

    fclose(input);
    return nb_sensor_data;
}

// ========================================================================== //
int main( int argc,char** argv )
{
    FILE *output, *output2;
    double angle_rad, angle_deg;
    Position startpos, position, position2;
    char filename[256];
    int i, x, y, test;
    int nb_sensor_data, cnt_scans;
    int timestamp, told, q1, q2, nq1, nq2;
    double m, v, vodo, thetarad, psidot, thetaradodo, psidotodo, psidotodo_old, vodo_old;
    cart_parameters_t params;

    params.r = 0.077;
    params.R = 0.165;
    params.inc = 2000;
    params.ratio = 1.0;

    cv::namedWindow("1",cv::WINDOW_NORMAL);

    // Read all the scans
    if(1 == argc)
      nb_sensor_data = read_sensor_data(TEST_FILENAME,sensor_data);
    else
      nb_sensor_data = read_sensor_data(argv[1],sensor_data);
    printf("sensor data = %d\n", nb_sensor_data);

    for (test = 0; test != 1; test++) {

        ts_map_init(&map);
        ts_map_init(&trajectory);
        output = fopen("test_trajectory.dat", "wt");
        position.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
        position.y = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
        position.theta = 0;
        startpos = position2 = position;
        //for (cnt_scans = 0; cnt_scans != nb_sensor_data; cnt_scans++) {
        for (cnt_scans = nb_sensor_data - 1; cnt_scans >= 0; cnt_scans--) {

            timestamp = sensor_data[cnt_scans].timestamp;
            nq1 = sensor_data[cnt_scans].q1;
            nq2 = sensor_data[cnt_scans].q2;

            // Manage robot position
            if (cnt_scans != 0) {
                psidotodo_old = psidotodo;
                vodo_old = vodo;
                m = params.r * M_PI / params.inc;
                vodo = m * (nq1 - q1 + (nq2 - q2) * params.ratio);
                thetarad = position.theta * M_PI / 180;
                position2 = position;
                position2.x += vodo * 1000 * cos(thetarad);
                position2.y += vodo * 1000 * sin(thetarad);
                psidotodo = (m * ((nq2 - q2) * params.ratio - nq1 + q1) / params.R) * 180 / M_PI;
                position2.theta += psidotodo;
                vodo *= 1000000.0 / (timestamp - told);
                psidotodo *= 1000000.0 / (timestamp - told);
                fprintf(output, "%d %d %lg %lg %lg %lg %lg", cnt_scans, timestamp - told, position2.x, position2.y, position2.theta, fabs(vodo), psidotodo);
            } else {
                psidotodo_old = psidotodo = 0;
                vodo_old = vodo = 0;
            }

            position2 = monte_carlo_move(&sensor_data[cnt_scans].scan, &map, &position, 0);
            v = sqrt((position2.x - position.x) * (position2.x - position.x) +
                    (position2.y - position.y) * (position2.y - position.y));
            psidot = position2.theta - position.theta;
            if (cnt_scans != 0) {
                v *= 1000.0 / (timestamp - told);
                psidot *= 1000000.0 / (timestamp - told);
            } else {
                v = 0;
                psidot = 0;
            }
            if (cnt_scans) {
                fprintf(output, " %lg %lg", v, psidot);
            }

            position = position2;
            printf("#%d : %lg %lg %lg\n", cnt_scans, position.x, position.y, position.theta);
            ts_map_update(&sensor_data[cnt_scans].scan, &map, &position, 50, TEST_HOLE_WIDTH);

            x = (int)floor(position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 254;

            if (0) {
                sprintf(filename, "test_lab_dist%04d.txt", cnt_scans);
                output2 = fopen(filename, "wt");
                position2 = position;
                for (i = 0; i < 500; i++) {
                    position2.x++;
                    fprintf(output2, "%d\n", ts_distance_scan_to_map(&sensor_data[cnt_scans].scan, &map, &position2));
                }
                fclose(output2);
            }
            {
                draw_scan(&sensor_data[cnt_scans].scan, &trajectory, &position);
                sprintf(filename, "test_lab%04d.pgm", cnt_scans);
                draw_map2(&map, &trajectory, 3500, 1600);
            }
            fprintf(output, "\n");
            q1 = nq1;
            q2 = nq2;
            told = timestamp;
        }
        fclose(output);

        // Record the map
        sprintf(filename, "test_lab_reverse%04d.pgm", test);
        record_map(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);
    }

    return 1;
}
