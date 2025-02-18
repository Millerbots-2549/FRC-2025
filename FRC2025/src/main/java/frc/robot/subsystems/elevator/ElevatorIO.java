// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean elevatorConnected = false;
        public boolean outOfBounds = false;

        public double velocityMetersPerSecond = 0.0;
        public double heightMeters = 0.0;
        public double heightSetpointMeters = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void applySetpointMeters(double position) {};

    public default void applyVelocityMetersPerSecond(double velocity) {};

    public default void applyDutyCycle(double output) {};

    public default void applyIntakeVelocity(double velocity) {};

    public default void applyIntakeDutyCycle(double output) {};

    public default void playMusic() {};

    public default void stopMusic() {};
}
