// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected;
        public Rotation2d yaw;
        public double yawVelocityRadPerSec;
        public double[] odometryTimestamps;
        public Rotation2d[] odometryYaw;
    }

    public default void updateInputs(GyroIOInputs inputs) {};
}
