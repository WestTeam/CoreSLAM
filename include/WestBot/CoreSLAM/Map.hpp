// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_MAP_HPP_
#define WESTBOT_CORESLAM_MAP_HPP_

#include "Defines.hpp"

namespace WestBot {
namespace CoreSLAM {

typedef unsigned short ts_map_pixel_t;

class Map
{
public:
    Map() = default;

    ts_map_pixel_t map[ TS_MAP_SIZE * TS_MAP_SIZE ];
};

}
}

#endif // WESTBOT_CORESLAM_MAP_HPP_
