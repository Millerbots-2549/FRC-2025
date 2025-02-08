// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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
        public boolean connected = false;
        public boolean calibrating = false;
        public Rotation2d yaw = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public Rotation3d rot3d = new Rotation3d();
        public Translation3d accel = new Translation3d();

        public double updateCount = 0;
        public double byteCount = 0;
    }

    public default void updateInputs(GyroIOInputs inputs) {};

    public default void zeroGyro(Rotation2d rotation) {};

    public default boolean printDisconnectErrorBytes(byte first_address, byte[] buffer) { return true; };
}
