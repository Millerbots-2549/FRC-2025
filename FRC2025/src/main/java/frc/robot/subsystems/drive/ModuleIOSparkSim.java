// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.SparkModuleConstants;
import frc.robot.util.SimulationUtils;

/**
 * This is an implementation of the {@link ModuleIO} interface which uses
 * MapleSim's {@link SwerveModuleSimulation}, as well as two {@link
 * SimulatedMotorController#GenericMotorController GenericMotorController}
 * simulations to figure out how the module should move in real life.
 * This is different from {@link ModuleIOSim}
 * 
 * <p><b>NOTE: </b> this is based on the old swerve modules, and needs to
 * be changed when we change the motors to Krakens.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class ModuleIOSparkSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController = new PIDController(
        SparkModuleConstants.driveSimP, 0, SparkModuleConstants.driveSimD);
    private final PIDController turnController = new PIDController(
        SparkModuleConstants.turnSimP, 0, SparkModuleConstants.turnSimD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSparkSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(DriveConstants.DRIVE_CURRENT_LIMIT));
        this.turnMotor = moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(DriveConstants.TURN_CURRENT_LIMIT));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
        driveAppliedVolts =
            driveFFVolts
                + driveController.calculate(
                    moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond));
        } else {
        driveController.reset();
        }
        if (turnClosedLoop) {
        turnAppliedVolts =
            turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
        turnController.reset();
        }

        // Update simulation state
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRadians = moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians);
        inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec =
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

        // Update odometry inputs
        inputs.odometryTimestamps = SimulationUtils.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRad =
            Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Units.Radians))
                .toArray();
        inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts =
            SparkModuleConstants.driveSimKs * Math.signum(velocityRadPerSec)
                + SparkModuleConstants.driveSimKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public ModuleGains getGains() { return new ModuleGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

    @Override
    public void setGains(ModuleGains gains) {}
}
