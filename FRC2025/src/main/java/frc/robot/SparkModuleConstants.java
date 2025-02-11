// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MathConstants;

/** Add your docs here. */
public class SparkModuleConstants {
    public static final double driveKp = 0.001;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.0789;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static final double turnKp = 0.314;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0;
    public static final double turnPIDMaxInput = MathConstants.TAU;

    public static final SparkBaseConfig driveConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT)
        .voltageCompensation(12.0)
        .apply(
            new EncoderConfig()
                .positionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2))
        .apply(
            new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(driveKp, 0.0, driveKd, 0.0))
        .apply(
            new SignalsConfig()
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int)(1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20));

    public static final SparkBaseConfig turnConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.TURN_CURRENT_LIMIT)
        .voltageCompensation(12.0)
        .apply(
            new EncoderConfig()
                .positionConversionFactor(DriveConstants.TURN_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DriveConstants.TURN_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2))
        .apply(
            new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pidf(turnKp, 0.0, turnKd, 0.0))
        .apply(
            new SignalsConfig()
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int)(1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20));

    public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    
    public record ModuleSpecConfig(
        int driveCanID,
        int turnCanID,
        int CANcoderID,
        Rotation2d CANcoderOffset,
        boolean invertDrive,
        boolean invertTurn
    ) {}

    public static final ModuleSpecConfig frontLeft =
        new ModuleSpecConfig(
            DriveConstants.FRONT_LEFT_DRIVE_ID,
            DriveConstants.FRONT_LEFT_TURN_ID,
            DriveConstants.FRONT_LEFT_CANCODER_ID,
            DriveConstants.FRONT_LEFT_ZERO_ROTATION,
            false, false);
    public static final ModuleSpecConfig frontRight =
        new ModuleSpecConfig(
            DriveConstants.FRONT_RIGHT_DRIVE_ID,
            DriveConstants.FRONT_RIGHT_TURN_ID,
            DriveConstants.FRONT_RIGHT_CANCODER_ID,
            DriveConstants.FRONT_RIGHT_ZERO_ROTATION,
            true, false);
    public static final ModuleSpecConfig backLeft =
        new ModuleSpecConfig(
            DriveConstants.BACK_LEFT_DRIVE_ID,
            DriveConstants.BACK_LEFT_TURN_ID,
            DriveConstants.BACK_LEFT_CANCODER_ID,
            DriveConstants.BACK_LEFT_ZERO_ROTATION,
            false, false);
    public static final ModuleSpecConfig backRight =
        new ModuleSpecConfig(
            DriveConstants.BACK_RIGHT_DRIVE_ID,
            DriveConstants.BACK_RIGHT_TURN_ID,
            DriveConstants.BACK_RIGHT_CANCODER_ID,
            DriveConstants.BACK_RIGHT_ZERO_ROTATION,
            true, false);
}
