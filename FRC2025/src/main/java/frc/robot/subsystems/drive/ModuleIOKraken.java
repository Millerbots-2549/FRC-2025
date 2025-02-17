// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.util.MotorUtils.*;

/** Add your docs here. */
public class ModuleIOKraken implements ModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder cancoder;

    private final VoltageOut voltageControl = new VoltageOut(0.0);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);

    private final TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0.0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOCControl = new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOCControl = new VelocityTorqueCurrentFOC(0.0);

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveVoltage;
    private final StatusSignal<Current> driveCurrent;

    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnVoltage;
    private final StatusSignal<Current> turnCurrent;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOKraken(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        driveMotor = new TalonFX(constants.DriveMotorId);
        turnMotor = new TalonFX(constants.SteerMotorId);
        cancoder = new CANcoder(constants.EncoderId);

        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

        var turnConfig = constants.SteerMotorInitialConfigs;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource =
            switch (constants.FeedbackSource) {
                case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
                case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
                case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
                default -> throw new RuntimeException("Unsupported feedback source");
            };
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.1;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnMotor.getPosition();
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor.getPosition());
        turnVelocity = turnMotor.getVelocity();
        turnVoltage = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveVoltage,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnVoltage,
            turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, cancoder);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        StatusCode driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVoltage, driveCurrent);
        StatusCode turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, driveVoltage, turnCurrent);
        StatusCode turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRadians = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.canCoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnVoltage.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double d) -> d).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double d) -> Units.rotationsToRadians(d)).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream().map((Double d) -> Rotation2d.fromRotations(d)).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
                case Voltage -> voltageControl.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentControl.withOutput(output);
            });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
                case Voltage -> voltageControl.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentControl.withOutput(output);
            });
    }

    @Override
    public void setDriveVelocity(double velocity) {
        double velocityRPS = Units.radiansToRotations(velocity);
        driveMotor.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
                case Voltage -> velocityVoltageControl.withVelocity(velocityRPS);
                case TorqueCurrentFOC -> velocityTorqueCurrentFOCControl.withVelocity(velocityRPS);
            });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnMotor.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
                case Voltage -> positionVoltageControl.withPosition(rotation.getRotations());
                case TorqueCurrentFOC -> positionTorqueCurrentFOCControl.withPosition(rotation.getRotations());
            });
    }
}
