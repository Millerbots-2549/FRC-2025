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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.SparkModuleConstants;
import frc.robot.SparkModuleConstants.ModuleSpecConfig;

/**
 * This is an implementation of the {@link ModuleIO} interface which uses
 * two {@link SparkMax Spark Max} motor controllers to control the wheel.
 * This class has two motors, one motor for turning and one for drive.
 * The swerve modules also use a CANCoder to detect the rotation of the
 * module, and the relative encoder on the turn motor should be used to
 * update the velocity input.
 * 
 * <p>If you want to change any of the module configurations, you can do
 * so in the {@link SparkModuleConstants} class.
 * 
 * <p><b>NOTE: </b> this is based on the old swerve modules, and needs to
 * be changed when we change the motors to Krakens.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class ModuleIOSpark implements ModuleIO {
    /** The offset of the CANCoder */
    private final Rotation2d zeroRotation;

    /** If the <b>drive</b> should be inverted */
    private final boolean invert;

    /** The drive motor as a {@link SparkBase} */
    private final SparkBase driveMotor;
    /** The turn motor as a {@link SparkBase} */
    private final SparkBase turnMotor;
    /** A relative encoder used by the drive motor */
    private final RelativeEncoder driveEncoder;
    /** A relative encoder used by the turn motor */
    private final RelativeEncoder turnEncoder;

    private final SparkBaseConfig driveConfig;
    private final SparkBaseConfig turnConfig;

    private ModuleGains gains;

    private final CANcoder canCoder;
    private final StatusSignal<Angle> turnAbsolutePosition;

    /**
     * The {@link PIDController} for the turn motor. We have to use a generic PIDController
     * Instead of a {@link SparkClosedLoopController} because otherwise it doesn't work for
     * some reason
     */
    private PIDController turnPID;
    /** The {@link SparkClosedLoopController closed loop controller} for the drive motor */
    private final SparkClosedLoopController driveController;

    // These are used by the odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    /**
     * Constructs the Swerve Module and initializes all of the hardware using the provided
     * {@link ModuleSpecConfig}. This implementation of {@link ModuleIO} uses two {@link SparkMax
     * Spark Max} motor controllers and a {@link CANcoder CAN Coder} as the turn encoder.
     * @param config The configuration for this module
     */
    public ModuleIOSpark(ModuleSpecConfig config) {
        zeroRotation = config.CANcoderOffset();

        // Setting up all of the hardware
        driveMotor = new SparkMax(config.driveCanID(), MotorType.kBrushless);
        turnMotor = new SparkMax(config.turnCanID(), MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        canCoder = new CANcoder(config.CANcoderID());

        // Setting up the turn PID controller
        turnPID = new PIDController(SparkModuleConstants.turnKp, 0, SparkModuleConstants.turnKd);
        turnPID.enableContinuousInput(SparkModuleConstants.turnPIDMinInput, SparkModuleConstants.turnPIDMaxInput);
        turnPID.setTolerance(10);
        
        driveController = driveMotor.getClosedLoopController();

        // Configuring the drive motor
        invert = config.invertDrive();
        driveConfig = SparkModuleConstants.driveConfig;
        driveConfig.inverted(config.invertDrive());
        tryUntilOk(driveMotor, 5, () ->
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        // Configuring the turn motor
        turnConfig = SparkModuleConstants.turnConfig;
        turnConfig.inverted(config.invertTurn());
        tryUntilOk(turnMotor, 5, () ->
            turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        turnEncoder.setPosition(0.0);

        // Configuring the CANCoder
        canCoder.getConfigurator().apply(SparkModuleConstants.canCoderConfig, 0.25);
        turnAbsolutePosition = canCoder.getAbsolutePosition();

        turnMotor.getEncoder().setPosition(
            canCoder.getPosition().getValueAsDouble() * DriveConstants.TURN_ENCODER_POSITION_FACTOR);

        // Creating all of the queues used by the odometry thread
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor, turnEncoder::getPosition);

        driveController.setReference(0, ControlType.kVelocity);

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
        // Updating the drive motor inputs
        sparkStickyFault = false;
        ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePositionRadians = value);
        ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = invert ? -value : value);
        ifOk(driveMotor, new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Updating the turn motor inputs
        sparkStickyFault = false;
        ifOk(
            turnMotor,
            this::getTurnPosition,
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnMotor, canCoder.getVelocity()::getValueAsDouble, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(turnMotor, new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        turnAbsolutePosition.refresh();
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity()) / DriveConstants.TURN_ENCODER_VELOCITY_FACTOR;
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

        // Updating the odometry inputs
        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        SmartDashboard.putNumber("CANCoder" + driveMotor.getDeviceId(), Units.rotationsToRadians(canCoder.getAbsolutePosition().getValueAsDouble()));
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
        double vel = invert ? -velocity : velocity; // Inverts the velocity based on the invert boolean
        // Calculating the feed forward volts using the formula V = K_s + K_v * v + K_a * a
        double ffVolts =
            SparkModuleConstants.driveKs * Math.signum(vel)
            + SparkModuleConstants.driveKv * vel;
        driveController.setReference(
            MathUtil.clamp(vel, -1.0, 1.0),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // We need to map the absolute position of the module to the correct range
        double absolutePosition =
            MathUtil.inputModulus(
                Units.rotationsToRadians(canCoder.getAbsolutePosition().getValueAsDouble()),
                SparkModuleConstants.turnPIDMinInput,
                SparkModuleConstants.turnPIDMaxInput);
        // Mapping the setpoint to the same range as the absolute position
        double setPoint =
            MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(),
                SparkModuleConstants.turnPIDMinInput,
                SparkModuleConstants.turnPIDMaxInput);
        // Makes sure the PID values are right, remove later
        turnPID.setP(SparkModuleConstants.turnKp);
        turnPID.setI(0.0);
        turnPID.setD(SparkModuleConstants.turnKd);
        turnPID.setTolerance(0.1, 0.1);
        // Used for debugging
        SmartDashboard.putNumber("Turn Error" + canCoder.getDeviceID(), absolutePosition);
        // Sets the motor speed
        double output = turnPID.calculate(absolutePosition, setPoint);
        output = MathUtil.clamp(output, -1.0, 1.0);
        turnMotor.set(output);
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
    
    public double getTurnPosition() {
        double turnPosition =
            MathUtil.inputModulus(
                canCoder.getPosition().getValueAsDouble() * DriveConstants.TURN_ENCODER_POSITION_FACTOR,
                SparkModuleConstants.turnPIDMinInput,
                SparkModuleConstants.turnPIDMaxInput);
        turnPosition %= MathConstants.TAU;
        return turnPosition;
    }

    public double getTurnVelocity() {
        double turnVelocity = turnMotor.getEncoder().getVelocity() / DriveConstants.TURN_ENCODER_VELOCITY_FACTOR;
        return turnVelocity;
    }
}
