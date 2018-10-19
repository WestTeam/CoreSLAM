// Copyright (c) 2018 All Rights Reserved WestBot

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <WestBot/CoreSLAM/CoreSlam.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

void WestBot::CoreSLAM::ts_save_map_pgm(
    Map* map,
    Map* overlay,
    char *filename,
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
