// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRadians = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public boolean canCoderConnected = true;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveOpenLoop(double output) {}

    public default void setTurnOpenLoop(double output) {}

    public default void setDriveVelocity(double velocity) {}

    public default void setTurnPosition (Rotation2d rotation) {}

    public default ModuleGains getGains() { return new ModuleGains(0, 0, 0, 0, 0, 0); }

    public default void setGains(ModuleGains gains) {}

    public record ModuleGains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
