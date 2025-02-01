// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

/** Add your docs here. */
public class ArmFeedForward {
    public double kS;
    public double kG;
    public double kV;
    public double kA;

    public ArmFeedForward(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        return kS * Math.signum(velocityRadPerSec)
            + kG * Math.cos(positionRadians)
            + kV * velocityRadPerSec
            + kA * accelRadPerSecSquared;
    }
    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }

    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return (maxVoltage - kS - Math.cos(angle) * kG - acceleration * kA) / kV;
    }

    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return (-maxVoltage + kS - Math.cos(angle) * kG - acceleration * kA) / kV;
    }

    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return (maxVoltage - kS * Math.signum(velocity) - Math.cos(angle) * kG - velocity * kV) / kA;
    }

    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, angle, velocity);
    }
}
