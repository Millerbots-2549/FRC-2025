// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface used to create different hardware and simulated implementations of
 * an individual swerve module on the robot.
 * 
 * <p>The inner class {@link ModuleIOInputs} is used to store all of the
 * inputs that will be logged to <a href="https://docs.advantagescope.com">
 * AdvantageScope</a> (Included in WPILib as of 2025, located in the WPILib
 * Tools folder)</p>
 * <ul>
 *  <li>The method {@link ModuleIO#updateInputs updateInputs} is used to update
 *     a given {@link ModuleIOInputs} object with the inputs calculated with this
 *     class. This should be changed for every implementation of this
 *     interface.</li>
 *  <li>The inputs with the suffix 'Connected' should be set to be always true in
 *     simulation. Otherwise, they should be set with a {@link Debouncer} with a
 *     debounce time of 0.5 seconds.</li>
 * </ul>
 * 
 * <p>The record {@link ModuleGains} contains PID values for the <b>drive motor</b>
 * and can be accessed through the getGains() method, and set using the method
 * setGains()
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
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

    /**
     * Runs the drive motor at a specific voltage (not using a PID controller)
     * @param output The voltage to set the motor at
     */
    public default void setDriveOpenLoop(double output) {}

    /**
     * Runs the turn motor at a specific voltage (not using a PID controller)
     * @param output The voltage to set the motor at.
     */
    public default void setTurnOpenLoop(double output) {}

    /**
     * Runs the drive motor to run at a specific velocity in <b>rotations per second</b>
     * @param velocity The velocity in <b>rotations per second</b>
     */
    public default void setDriveVelocity(double velocity) {}

    /**
     * Turns the module to a setpoint.
     * @param rotation The setpoint as a {@link Rotation2d}
     */
    public default void setTurnPosition (Rotation2d rotation) {}
}
