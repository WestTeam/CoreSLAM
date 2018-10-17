// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_DEFINES_HPP_
#define WESTBOT_CORESLAM_DEFINES_HPP_

namespace WestBot {
namespace CoreSLAM {

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

}
}

#endif // WESTBOT_CORESLAM_DEFINES_HPP_
