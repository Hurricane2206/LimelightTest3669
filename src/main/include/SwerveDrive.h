#pragma once
#include "AHRS.h"
#include "SwerveModule.h"

// runs a swerve chassis
class SwerveDrive
{

private:
    // array of four swerve module objects with their positions relative to the center specified
    SwerveModule modules[4] =
        {
            SwerveModule{11, 31, 21, {-17.75, 25}},
            SwerveModule{12, 32, 22, {-17.75, -25}},
            SwerveModule{13, 33, 23, {17.75, 25}},
            SwerveModule{14, 34, 24, {17.75, -25}}};

    // NavX V2 object
    AHRS navx{frc::SPI::Port::kMXP};

    Vector currentFieldRate;         // stores current field rate setpoint
    float currentAngularRate = 0;    // stores current angular rate setpoint
    float fieldAngle;                // current angle on the field
    Vector fieldPosition;            // current position on the field relative to the starting position
    Vector fieldPositionChange;      // stores the robot's change in position since the last cycle
    float fastestModule;             // stores the speed of the fastest swerve module.
    float moduleWheelSpeed;          // stores each module's wheel speed for comparison
    Vector positionalAccelIncrement; // stores a vector specifying how quickly to change the drive velocity
    float angularAccelIncrement;     // stores a float specifying how quickly to change the rotation rate

public:
    // initialize the swerve modules and zero the NavX yaw
    void initialize()
    {
        for (int i = 0; i < 4; i++)
        {
            modules[i].initialize();
        }
        navx.ZeroYaw();
    }

    void Set(Vector targetFieldRate, float angularRateTarget)
    {
        // set the current field angle to the gyro angle + the starting angle
        fieldAngle = angleSum(navx.GetYaw(), parameters.startingAngle);
        // robot-orient the drive command
        targetFieldRate.rotateCW(-fieldAngle);
        // keep the module speeds <= 1
        normalizeSwerveRate(targetFieldRate, angularRateTarget);
        // field-orient the drive rate command again
        targetFieldRate.rotateCW(fieldAngle);
        // find the robot field drive rate error
        positionalAccelIncrement = targetFieldRate.getSubtracted(currentFieldRate).getScaled(0.5);
        angularAccelIncrement = (angularRateTarget - currentAngularRate) * 0.5;
        // limit increments to max acceleration rate
        if (t2D::abs(positionalAccelIncrement) > parameters.maxPercentChangePerCycle)
        {
            positionalAccelIncrement.scale(parameters.maxPercentChangePerCycle / t2D::abs(positionalAccelIncrement));
        }
        if (std::abs(angularAccelIncrement) > parameters.maxPercentChangePerCycle)
        {
            angularAccelIncrement *= parameters.maxPercentChangePerCycle / std::abs(angularAccelIncrement);
        }

        // increment from the current field rate
        currentFieldRate.add(positionalAccelIncrement);
        currentAngularRate += angularAccelIncrement;
        // reset position change vector before recalculating
        fieldPositionChange.reset();
        // drive the modules and average module position change
        for (int i = 0; i < 4; i++)
        {
            modules[i].Set(currentFieldRate.getRotatedCW(-fieldAngle), currentAngularRate);
            fieldPositionChange.add(modules[i].getPositionChangeVector());
        }
        // orient the position change vector in the direction of robot motion
        fieldPositionChange.rotateCW(fieldAngle);
        // average the position change of all four swerve modules
        fieldPositionChange.divide(4);
        // add the change in position over this cycle to the running total
        fieldPosition.add(fieldPositionChange.getDivided(50));
    }

    // limit the driving inputs to physically achievable values
    void normalizeSwerveRate(Vector &driveRate, float &currentAngularRate)
    {
        fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            moduleWheelSpeed = t2D::abs(modules[i].getModuleVector(driveRate, currentAngularRate));
            if (moduleWheelSpeed > fastestModule)
            {
                fastestModule = moduleWheelSpeed;
            }
        }
        driveRate.divide(fastestModule);
        currentAngularRate /= fastestModule;
    }

    // get the robot's absolute position on the field
    Vector getFieldPosition()
    {
        return parameters.startingPosition.getAdded(fieldPosition);
    }

    float getFieldAngle()
    {
        return fieldAngle;
    }
} swerve;