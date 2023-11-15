#pragma once
#include "Parameters.h"
#include "ctre/phoenixpro/TalonFX.hpp"
using namespace ctre::phoenixpro;

// control a Falcon500 brushless motor
class Falcon500 {
private:
    hardware::TalonFX *motor;
    controls::TorqueCurrentFOC accelerationController{0_A};
    controls::VelocityTorqueCurrentFOC velocityContoller{0_tps, 0_A, 1, false};
    float const maxRotationsPerSecond = 101;

public:
    Falcon500(int canID) {
        motor = new hardware::TalonFX(canID, "rio");
    }

    void initialize()
    {
        configs::TalonFXConfiguration configs{};
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;  // Peak output of 40 amps
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40; // Peak output of 40 amps
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
        motor->GetConfigurator().Apply(configs);
    }

    void SetAcceleration(float amperage) {
        motor->SetControl(accelerationController.WithOutput(amperage * 1_A));
    }

    void SetVelocity(float percentOfMaxVelocity) {
        auto frictionTorque = (percentOfMaxVelocity > 0) ? 1_A : -1_A;
        motor->SetControl(velocityContoller.WithVelocity(percentOfMaxVelocity * maxRotationsPerSecond * 1_tps).WithFeedForward(frictionTorque));
    }

    void Set(float speed) {
        motor->Set(speed);
    }

    float getPosition() {
        return motor->GetPosition().GetValue().value();
    }

    float getPercentOfMaxVelocity() {
        return motor->GetVelocity().GetValue().value() / maxRotationsPerSecond;
    }
};