// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface used to create different hardware and simulated implementations of
 * the gyro used on this robot.
 * 
 * <p>The inner class {@link GyroIOInputs} is used to store all of the
 * inputs that will be logged to <a href="https://docs.advantagescope.com">
 * AdvantageScope</a> (Included in WPILib as of 2025, located in the WPILib
 * Tools folder)</p>
 * <ul>
 *  <li>The method {@link GyroIO#updateInputs updateInputs} is used to update
 *     a given {@link GyroIOInputs} object with the inputs calculated with this
 *     class. This should be changed for every implementation of this
 *     interface.</li>
 * </ul>
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
*/
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
