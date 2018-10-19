// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_LOOPCLOSING_HPP_
#define WESTBOT_CORESLAM_LOOPCLOSING_HPP_

namespace WestBot {
namespace CoreSLAM {

class LoopClosing
{
public:
    LoopClosing() = default;

    //Position ts_close_loop_position(
    //    State* state,
    //    SensorData* sensor_data,
    //    Map* loop_close_map,
    //    Position* start_position,
    //    int* q );
    //
    //void ts_close_loop_trajectory(
    //    SensorData* sensor_data,
    //    int maxscans,
    //    Position* startpos,
    //    Position* close_loop_position );
};

}
}

#endif // WESTBOT_CORESLAM_LOOPCLOSING_HPP_
