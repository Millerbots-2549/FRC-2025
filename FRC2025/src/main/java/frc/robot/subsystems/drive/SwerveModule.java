// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;

    private SwerveModulePosition[] odometryPositions;

    public SwerveModule(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert =
            new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert =
            new Alert(
                "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    }
    
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RAIDUS_METERS;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
    }

    public void apply(SwerveModuleState state) {
        io.apply(state);
    }

    public void applyCharacterization(Rotation2d turn, double driveVoltage) {
        io.applyCharacterization(turn, driveVoltage);
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * DriveConstants.WHEEL_RAIDUS_METERS;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RAIDUS_METERS;
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRadians;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
