// Copyright (c) 2018 All Rights Reserved WestBot

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>		// Use the C99 official header

#include <WestBot/CoreSLAM/Map.hpp>
#include <WestBot/CoreSLAM/Position.hpp>
#include <WestBot/CoreSLAM/Scan.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

void Scan::ts_draw_scan( Map* map, Position* pos )
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != nb_points; i++) {
        if (value[i] != TS_NO_OBSTACLE) {
            x2p = c * x[i] - s * y[i];
            y2p = s * x[i] + c * y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE;
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE)
                map->map[y2 * TS_MAP_SIZE + x2] = 0;
        }
    }
}

void Scan::ts_draw_scan_RGB(
    Map* map,
    Position* pos,
    unsigned char* pixmap,
    int scale,
    int reversey )
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;
    int color;
    unsigned char *ptr;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i < nb_points; i++) {
        if (value[i] != TS_NO_OBSTACLE) {
            x2p = c * x[i] - s * y[i];
            y2p = s * x[i] + c * y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE;
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE) {
                color = map->map[y2 * TS_MAP_SIZE + x2] >> 8;
                if (reversey)
                    ptr = &pixmap[((TS_MAP_SIZE - 1 - y2) / scale * TS_MAP_SIZE / scale + x2 / scale) * 3];
                else
                    ptr = &pixmap[(y2 / scale * TS_MAP_SIZE / scale  + x2 / scale) * 3];
                ptr[0] = 255 - color;
                ptr[1] = 0;
                ptr[2] = color;
            }
        }
    }
}
