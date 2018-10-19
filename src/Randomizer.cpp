// Copyright (c) 2018 All Rights Reserved WestBot

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <WestBot/CoreSLAM/Randomizer.hpp>

using namespace WestBot;
using namespace WestBot::CoreSLAM;

namespace
{
    unsigned long SHR3( WestBot::CoreSLAM::Randomizer* d )
    {
        d->jz=d->jsr;
        d->jsr^=(d->jsr<<13);
        d->jsr^=(d->jsr>>17);
        d->jsr^=(d->jsr<<5);
        return d->jz+d->jsr;
    }

    double UNI( WestBot::CoreSLAM::Randomizer* d )
    {
        return .5 + (signed)SHR3(d) * .2328306e-9;
    }
}

Randomizer::Randomizer()
{
}

double Randomizer::ts_random_normal_fix()
{
    const double r = 3.442620; 	// The starting of the right tail
    static double x, y;
    for(;;)
    {
        x=hz * wn[ iz ];
        if( iz == 0 )
        { // iz==0, handle the base strip
            do
            {
                x=-log( UNI( this ) ) * 0.2904764;
                // .2904764 is 1/r
                y=-log( UNI( this ) );
            } while(y+y<x*x);
            return( hz > 0 ) ? r + x : -r - x;
        }

        // iz>0, handle the wedges of other strips
        if( fn[ iz ]+UNI( this )*( fn[iz-1]-fn[iz]) < exp(-.5*x*x) )
            return x;
        // Start all over
        hz=::SHR3( this );
        iz=hz&127;
        if((unsigned long)abs(hz)<kn[iz])
            return (hz*wn[iz]);
    }
}

double Randomizer::ts_random_normal( double m, double s )
{
    double x;
    hz = ::SHR3( this );
    iz = hz & 127;
    x= ((unsigned long)abs(hz) < kn[iz])? hz * wn[iz] : ts_random_normal_fix(); // Generic version
    return x * s + m ;
};

void Randomizer::ts_random_init( unsigned long jsrseed )
{
    const double m1 = 2147483648.0;

    double dn=3.442619855899, tn=dn, vn=9.91256303526217e-3, q;
    int i;
    jsr=jsrseed;

    // Set up tables for Normal
    q=vn/exp(-.5*dn*dn);
    kn[0]=(int)((dn/q)*m1);
    kn[1]=0;
    wn[0]=q/m1;
    wnt[0]=q;
    wn[127]=dn/m1;
    wnt[127]=dn;
    fn[0]=1.;
    fn[127]=exp(-.5*dn*dn);
    for(i=126;i>=1;i--) {
        dn=sqrt(-2.*log(vn/dn+exp(-.5*dn*dn)));
        kn[i+1]=(int)((dn/tn)*m1);		  tn=dn;
        fn[i]=exp(-.5*dn*dn);
        wn[i]=dn/m1;
        wnt[i]=dn;
    }
}

double Randomizer::ts_random()
{
    return ::UNI( this );
}

long Randomizer::ts_random_int( long min, long max )
{
    // Output random integer in the interval min <= x <= max
    long r;
    r = (long)((max - min + 1) * ts_random()) + min; // Multiply interval with random and truncate
    if (r > max) r = max;
    if (max < min) return 0x80000000;
    return r;
}

Position Randomizer::ts_monte_carlo_search(
    Scan* scan,
    Map* map,
    Position* start_pos,
    double sigma_xy,
    double sigma_theta,
    int stop,
    int* bd )
{
    Position currentpos, bestpos, lastbestpos;
    int currentdist;
    int bestdist, lastbestdist;
    int counter = 0, debug = 0;

    if (stop < 0) {
        debug = 1;
        stop = -stop;
    }
    currentpos = bestpos = lastbestpos = *start_pos;
    currentdist = map->ts_distance_scan_to_map(scan, &currentpos);
    bestdist = lastbestdist = currentdist;

    do {
	currentpos = lastbestpos;
	currentpos.x = ts_random_normal(currentpos.x, sigma_xy);
	currentpos.y = ts_random_normal(currentpos.y, sigma_xy);
	currentpos.theta = ts_random_normal(currentpos.theta, sigma_theta);

	currentdist = map->ts_distance_scan_to_map(scan, &currentpos);

	if (currentdist < bestdist) {
	    bestdist = currentdist;
	    bestpos = currentpos;
            if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n", bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
	} else {
	    counter++;
	}
        if (counter > stop / 3) {
            if (bestdist < lastbestdist) {
                lastbestpos = bestpos;
                lastbestdist = bestdist;
                counter = 0;
                sigma_xy *= 0.5;
                sigma_theta *= 0.5;
            }
        }
    } while (counter < stop);
    if (bd)
        *bd = bestdist;
    return bestpos;
}
