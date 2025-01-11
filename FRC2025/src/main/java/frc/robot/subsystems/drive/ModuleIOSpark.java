// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

import static frc.robot.util.MotorUtils.*;

/** Add your docs here. */
public class ModuleIOSpark implements ModuleIO {
    private final Rotation2d zeroRotation;

    private final SparkBase driveMotor;
    private final SparkBase turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final CANcoder canCoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(int module) {
        zeroRotation =
            switch (module) {
                case 0 -> DriveConstants.FRONT_LEFT_ZERO_ROTATION;
                case 1 -> DriveConstants.FRONT_RIGHT_ZERO_ROTATION;
                case 2 -> DriveConstants.BACK_LEFT_ZERO_ROTATION;
                case 3 -> DriveConstants.BACK_RIGHT_ZERO_ROTATION;
                default -> new Rotation2d();
            };
        driveMotor =
            new SparkMax(
                switch (module) {
                    case 0 -> DriveConstants.FRONT_LEFT_DRIVE_ID;
                    case 1 -> DriveConstants.FRONT_RIGHT_DRIVE_ID;
                    case 2 -> DriveConstants.BACK_LEFT_DRIVE_ID;
                    case 3 -> DriveConstants.BACK_RIGHT_DRIVE_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnMotor =
            new SparkMax(
                switch (module) {
                    case 0 -> DriveConstants.FRONT_LEFT_TURN_ID;
                    case 1 -> DriveConstants.FRONT_RIGHT_TURN_ID;
                    case 2 -> DriveConstants.BACK_LEFT_TURN_ID;
                    case 3 -> DriveConstants.BACK_RIGHT_TURN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkFlexConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        driveConfig
            .encoder
            .positionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DriveConstants.DRIVE_P, 0.0,
                DriveConstants.DRIVE_D, 0.0);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            driveMotor,
            5,
            () ->
                driveMotor.configure(
                    driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(DriveConstants.TURN_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.TURN_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        turnConfig
            .encoder
            .inverted(DriveConstants.TURN_ENCODER_INVERTED)
            .positionConversionFactor(DriveConstants.TURN_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DriveConstants.TURN_ENCODER_VELOCITY_FACTOR);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(DriveConstants.TURN_PID_MIN_INPUT, DriveConstants.TURN_PID_MAX_INPUT)
            .pidf(DriveConstants.TURN_P, 0.0, DriveConstants.TURN_D, 0.0);
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            turnMotor,
            5,
            () ->
                turnMotor.configure(
                    turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create odometry queues
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue =
            OdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        turnPositionQueue =
            OdometryThread.getInstance().registerSignal(turnMotor, turnEncoder::getPosition);

        int ccid = switch (module) {
            case 0 -> DriveConstants.FRONT_LEFT_CANCODER_ID;
            case 1 -> DriveConstants.FRONT_RIGHT_CANCODER_ID;
            case 2 -> DriveConstants.BACK_LEFT_CANCODER_ID;
            case 3 -> DriveConstants.BACK_RIGHT_CANCODER_ID;
            default -> 0;
        };
        canCoder = new CANcoder(
            ccid
        );

        // TODO: configure CANcoder
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void apply(SwerveModuleState state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'apply'");
    }

    @Override
    public void applyCharacterization(Rotation2d turn, double driveVolts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'applyCharacterization'");
    }

    @Override
    public void setIdleMode(IdleMode driveIdleMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public void setIdleMode(IdleMode driveIdleMode, IdleMode turnIdleMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public SwerveModuleState getTargetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetState'");
    }

    @Override
    public void resetPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }
    
}
