// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.ModuleIO.ModuleGains;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class SwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert canCoderDisconnectedAlert;

    private SwerveModulePosition[] odometryPositions;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

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
        canCoderDisconnectedAlert =
            new Alert(
                "Disconnected CANcoder on module " + Integer.toString(index) + ".", AlertType.kError);

        ModuleGains gains = io.getGains();

        kP = new LoggedTunableNumber("Drive/" + index + "/Gains/kP", gains.kP());
        kI = new LoggedTunableNumber("Drive/" + index + "/Gains/kI", gains.kI());
        kD = new LoggedTunableNumber("Drive/" + index + "/Gains/kD", gains.kD());
        kS = new LoggedTunableNumber("Drive/" + index + "/Gains/kS", gains.kS());
        kV = new LoggedTunableNumber("Drive/" + index + "/Gains/kV", gains.kV());
        kA = new LoggedTunableNumber("Drive/" + index + "/Gains/kA", gains.kA());
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
        canCoderDisconnectedAlert.set(!inputs.canCoderConnected);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (values) -> {
                io.setGains(
                    new ModuleGains(values[0], values[1], values[2], values[3], values[4], values[5]));
            },
            kP, kI, kD, kS, kV, kA);
    }

    public void apply(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_RAIDUS_METERS);
        io.setTurnPosition(state.angle);
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * DriveConstants.WHEEL_RAIDUS_METERS;
    }

    public double getVelocityMetersPerSecond() {
        return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RAIDUS_METERS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
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
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
