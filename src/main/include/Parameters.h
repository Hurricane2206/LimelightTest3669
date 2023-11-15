#pragma once
#include "Vector.h"

// parameters for robot movement and autonomous
struct Parameters
{  
    float const ampsForRobotAccel = 50;
    float const wheelDiameter = 3.9;
    float const driveMotorInchesPerRotation = (M_PI * wheelDiameter / 6.75);
    float const maxPercentChangePerCycle = 0.035;
    float const autoMaxDriveRate = 0.2; // max drive rate for autonomous
    float const autoMaxRotationRate = 0.2; // max rotation rate for autonomous
    float const autoPositionP = 0.01; // proportional constant for autonomous position error
    float const autoAngleP = 0.005; // proportional constant for autonomous angle error

    // swerve presets
    Vector startingPosition = {0, 0};
    float startingAngle = 0;
} parameters;