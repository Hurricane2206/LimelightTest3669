#pragma once;
#include "SwerveDrive.h"

class SwervePIDController
{
  private:
    Vector positionError;
    Vector positionPIDOutput;
    float angleError = 0;
    float anglePIDOutput = 0;

  public:
    // drives the swerve drive toward a point
    bool driveToward(Vector targetPostition, float targetAngle, float positionTolerance = 2, float angleTolerance = 5)
    {
        positionError = targetPostition.getSubtracted(swerve.getFieldPosition());
        angleError = angleDifference(swerve.getFieldAngle(), targetAngle);
        positionPIDOutput = positionError.getScaled(parameters.autoPositionP);
        anglePIDOutput = angleError * parameters.autoAngleP;
        if (t2D::abs(positionPIDOutput) > parameters.autoMaxDriveRate)
        {
            positionPIDOutput.scale(parameters.autoMaxDriveRate/t2D::abs(positionPIDOutput));
        }
        if (std::abs(anglePIDOutput) > parameters.autoMaxRotationRate)
        {
            anglePIDOutput *= parameters.autoMaxRotationRate/std::abs(anglePIDOutput);
        }
        swerve.Set(positionPIDOutput, anglePIDOutput);
        return (t2D::abs(positionError) < positionTolerance) && (std::abs(angleError) < angleTolerance);
    }
}swerveAutoController;