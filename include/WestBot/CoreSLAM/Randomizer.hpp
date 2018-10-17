// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_RANDOMIZER_HPP_
#define WESTBOT_CORESLAM_RANDOMIZER_HPP_

namespace WestBot {
namespace CoreSLAM {

struct Randomizer
{
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
