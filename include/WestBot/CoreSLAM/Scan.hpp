// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_SCAN_HPP_
#define WESTBOT_CORESLAM_SCAN_HPP_

#include "Defines.hpp"

namespace WestBot {
namespace CoreSLAM {

class Scan
{
public:
    Scan() = default;

    double x[ TS_SCAN_SIZE ];
    double y[ TS_SCAN_SIZE ];
    int value[ TS_SCAN_SIZE ];
    int nb_points;
};

}
}

#endif // WESTBOT_CORESLAM_SCAN_HPP_
