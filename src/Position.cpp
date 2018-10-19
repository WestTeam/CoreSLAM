// Copyright (c) 2018 All Rights Reserved WestBot

#include <math.h>

#include <WestBot/CoreSLAM/Position.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

double Position::ts_distance( Position* pos1, Position* pos2 )
{
    return
        sqrt( ( pos1->x - pos2->x ) *
        ( pos1->x - pos2->x ) +
        ( pos1->y - pos2->y ) *
        ( pos1->y - pos2->y ) );
}
