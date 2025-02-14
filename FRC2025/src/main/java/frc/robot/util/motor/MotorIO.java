// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected;

        public double position;
        public double velocity;
        public double appliedVolts;
        public double statorCurrentAmps;
        public double supplyCurrentAmps;
        public double motorTemperature;
    }

    public static enum MotorNeutralMode {
        BRAKE,
        COAST
    }

    default public void updateInputs(MotorIOInputs inputs) {}

    default public void applyVelocity(double velocity) {}

    default public void applyVoltage(double voltage) {}

    default public void applyDutyCycle(double output) {}

    default public void applyProfileSetpoint(double setpoint) {}

    default public void applySetpoint(double setpoint) {}

    default public void setNeutralMode(MotorNeutralMode neutralMode) {}

    default public void setSoftLimits(boolean forward, boolean reverse) {}

    default public void zeroPosition() {}

    default public void setInternalPosition(double position) {}
}
