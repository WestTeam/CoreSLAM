// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_RANDOMIZER_HPP_
#define WESTBOT_CORESLAM_RANDOMIZER_HPP_

namespace WestBot {
namespace CoreSLAM {

class Map;
class Position;
class Scan;

class Randomizer
{
public:
    Randomizer();

    double ts_random_normal_fix();

    double ts_random_normal(double m, double s );

    void ts_random_init( unsigned long jsrseed );

    double ts_random();

    long ts_random_int( long min, long max );

    Position ts_monte_carlo_search(
        Scan* scan,
        Map* map,
        Position* start_pos,
        double sigma_xy,
        double sigma_theta,
        int stop,
        int* bestdist );

    unsigned long jz;
    unsigned long jsr;
    long hz;
    unsigned long iz;
    unsigned long kn[ 128 ];
    double wnt[ 128 ];
    double wn[ 128 ];
    double fn[ 128 ];
};

}
}

#endif // WESTBOT_CORESLAM_RANDOMIZER_HPP_
