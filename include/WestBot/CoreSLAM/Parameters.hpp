// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_CORESLAM_PARAMETERS_HPP_
#define WESTBOT_CORESLAM_PARAMETERS_HPP_

namespace WestBot {
namespace CoreSLAM {

class RobotParameters
{
public:
    RobotParameters() = default;

    double wheelRadius() const
    {
        return _wheelRadius;
    }

    double halfWheelAxisLength() const
    {
        return _halfWheelAxixLength;
    }

    int inc() const
    {
        return _inc;
    }

    double ratio() const
    {
        return _ratio;
    }

private:
    double _wheelRadius;	     // length wheels' radius
    double _halfWheelAxixLength; // half the wheels' axis length
    int _inc;	                 // wheels' counters increments per turn
    double _ratio;               // ratio between left and right wheel
};

class LaserParameters
{
public:
    LaserParameters() = default;

    double offset() const
    {
        return _offset;
    }

    int scanSize() const
    {
        return _scanSize;
    }

    int angleMin() const
    {
        return _angleMin;
    }

    int angleMax() const
    {
        return _angleMax;
    }

    int detectionMargin() const
    {
        return _detectionMargin;
    }

    double distanceNoDetection() const
    {
        return _distanceNoDetection;
    }

private:
    double _offset;  // position of the laser wrt center of rotation
    int _scanSize;  // number of points per scan
    int _angleMin;  // start angle for scan
    int _angleMax;  // end angle for scan
    int _detectionMargin; // first scan element to consider
    double _distanceNoDetection; // default value when the laser returns 0
};

}
}

#endif // WESTBOT_CORESLAM_PARAMETERS_HPP_
