#pragma once
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "Falcon500.h"
#include "angleOptimization.h"

class SwerveModule
{
private:
    Vector steeringVector; // module drive vector for steering the robot clockwise
    Falcon500 *driveMotor;
    rev::CANSparkMax *turningMotor;
    CANCoder *wheelAngleEncoder;
    float lastPosition = 0;
    Vector positionChangeVector;
    float currentWheelAngle;
    Vector moduleTargetVelocity;
    float error;
    float driveMotorVelocity;
    float currentPosition;

public:
    SwerveModule(int driveMotorCANID, int turningMotorCANID, int canCoderID, Vector position)
    {
        driveMotor = new Falcon500{driveMotorCANID};
        turningMotor = new rev::CANSparkMax{turningMotorCANID, rev::CANSparkMax::MotorType::kBrushless};
        wheelAngleEncoder = new CANCoder{canCoderID};
        // calculate the steering vector
        steeringVector = position;
        steeringVector.rotateCW(90);
        steeringVector.divide(t2D::abs(steeringVector));
    }

    // initialize the drive motor and invert the turning motor
    void initialize()
    {
        driveMotor->initialize();
        turningMotor->SetInverted(true);
        turningMotor->BurnFlash();
    }

    // calculate the swerve module vector
    Vector getModuleVector(Vector driveRate, float angularRate)
    {
        return driveRate.getAdded(steeringVector.getScaled(angularRate));
    }

    void Set(Vector driveRate, float angularRate)
    {
        // find the current wheel angle
        currentWheelAngle = wheelAngleEncoder->GetAbsolutePosition();
        // find the module target velocity
        moduleTargetVelocity = getModuleVector(driveRate, angularRate);
        // find the wheel's error from it's target angle
        error = angleDifference(moduleTargetVelocity.getAngle(), currentWheelAngle);
        // find the drive motor velocity
        driveMotorVelocity = t2D::abs(moduleTargetVelocity);
        // reverse the wheel direction if it is more efficient
        if (std::abs(error) > 90)
        {
            driveMotorVelocity = -driveMotorVelocity;
            error = angleSum(error, 180);
        }
        driveMotor->SetVelocity(driveMotorVelocity);
        // set the turning motor to a speed proportional to its error
        turningMotor->Set(error / 180);
        // find the delta position change since last Set() call
        currentPosition = driveMotor->getPosition();
        positionChangeVector = Vector{0, (currentPosition - lastPosition) * parameters.driveMotorInchesPerRotation}.getRotatedCW(currentWheelAngle);
        lastPosition = currentPosition;
    }

    // gets this module's position change, useful for calculating robot position
    Vector getPositionChangeVector()
    {
        return positionChangeVector;
    }
};