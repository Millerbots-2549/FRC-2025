// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.MotorUtils.ifOk;
import static frc.robot.util.MotorUtils.sparkStickyFault;
import static frc.robot.util.MotorUtils.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SparkModuleConstants;
import frc.robot.SparkModuleConstants.ModuleSpecConfig;

/** Add your docs here. */
public class ModuleIOSpark implements ModuleIO {
    private final SparkBase driveMotor;
    private final SparkBase turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkBaseConfig driveConfig;
    private final SparkBaseConfig turnConfig;

    private ModuleGains gains;

    private final CANcoder canCoder;
    private final StatusSignal<Angle> turnAbsolutePosition;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(ModuleSpecConfig config) {
        driveMotor = new SparkMax(config.driveCanID(), MotorType.kBrushless);
        turnMotor = new SparkMax(config.turnCanID(), MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        canCoder = new CANcoder(config.CANcoderID());

        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        driveConfig = SparkModuleConstants.driveConfig;
        driveConfig.inverted(config.invertDrive());
        tryUntilOk(driveMotor, 5, () ->
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        CANcoderConfiguration canCoderConfig = SparkModuleConstants.canCoderConfig;
        canCoderConfig.MagnetSensor.withMagnetOffset(config.CANcoderOffset());
        tryUntilOk(5, () -> canCoder.getConfigurator().apply(canCoderConfig, 0.25));

        turnConfig = SparkModuleConstants.turnConfig;
        turnConfig.inverted(config.invertTurn());
        tryUntilOk(turnMotor, 5, () ->
            turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(turnMotor, 5, () ->
            turnEncoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble()));

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor, turnEncoder::getPosition);

        turnAbsolutePosition = canCoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

        gains = new ModuleGains(
            SparkModuleConstants.driveKp,
            0.0,
            SparkModuleConstants.driveKd,
            SparkModuleConstants.driveKs,
            SparkModuleConstants.driveKv,
            0.0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePositionRadians = value);
        ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
            driveMotor, 
            new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
            turnMotor,
            turnEncoder::getPosition,
            (value) -> inputs.turnPosition = new Rotation2d(value));
        ifOk(turnMotor, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
            turnMotor, 
            new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        inputs.turnAbsolutePosition = Rotation2d.fromRadians(canCoder.getAbsolutePosition().getValueAsDouble());

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocity) {
        double ffVolts =
            SparkModuleConstants.driveKs * Math.signum(velocity)
            + SparkModuleConstants.driveKv * velocity;
        driveController.setReference(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setPoint =
            MathUtil.inputModulus(
                rotation.getRadians(),
                SparkModuleConstants.turnPIDMinInput,
                SparkModuleConstants.turnPIDMaxInput);
        turnController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public ModuleGains getGains() {
        return gains;
    }

    @Override
    public void setGains(ModuleGains gains) {
        this.gains = gains;

        tryUntilOk(
            driveMotor,
            5,
            () -> driveMotor.configure(
                driveConfig.apply(
                    new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), gains.kV())),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters));
    }
    
}
