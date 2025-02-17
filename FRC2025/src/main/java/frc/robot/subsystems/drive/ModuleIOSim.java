// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.Constants.MathConstants.PI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

/**
 * This is an implementation of the {@link ModuleIO} interface which uses
 * two {@link DCMotorSim} objects to simulate how the module <i>should</i>
 * move. This class uses preset gains in declared locally in this class.
 * 
 * <p><b>NOTE: </b> this is based on the old swerve modules, and needs to
 * be changed when we change the motors to Krakens.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 * @see ModuleIO
 */
public class ModuleIOSim implements ModuleIO {
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035;
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, DriveConstants.DRIVE_INERTIA, DriveConstants.DRIVE_GEAR_RATIO),
            DRIVE_GEARBOX);
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURN_GEARBOX, DriveConstants.TURN_INERTIA, DriveConstants.TURN_GEAR_RATIO),
            TURN_GEARBOX);
        
        turnController.enableContinuousInput(-PI, PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        inputs.driveConnected = true;
        inputs.drivePositionRadians = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.canCoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRadians};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
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
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
