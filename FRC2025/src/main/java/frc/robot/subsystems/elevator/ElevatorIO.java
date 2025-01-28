// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean elevatorConnected = true;

        public double elevatorHeightMeters = 0.0;
        public double elevatorVelocityMetersPerSecond = 0.0;
    }

    public default void periodic() {}

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setHeight(double height) {}

    public default void setVelocity(double velocity) {}
}
