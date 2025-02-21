// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.descorer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface DescorerIO {
    @AutoLog
    public static class DescorerIOInputs {
        public boolean wristConnected = false;

        public double wristVelocity = 0.0;
        public double wristAppliedVolts = 0.0;
        public double wristCurrent = 0.0;
        public Rotation2d wristPosition = Rotation2d.kZero;
    }

    public default void updateInputs(DescorerIOInputs inputs) {};

    public default void applyOpenLoop(double output) {};

    public default void applyVelocity(double velocity) {};

    public default void applySetpoint(Rotation2d setpoint) {};

    public default Rotation2d getCurrentSetpoint() { return new Rotation2d(); };
}
