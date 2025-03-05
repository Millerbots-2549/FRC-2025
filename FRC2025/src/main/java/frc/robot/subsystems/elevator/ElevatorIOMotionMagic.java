// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ElevatorIOMotionMagic implements ElevatorIO {
    //#region constants
    private static final double ELEVATOR_TOP = 1.0;
    private static final double ELEVATOR_BOTTOM = 0.0;

    private static final double SPROCKET_DIAMETER = 5.0;
    private static final double MOTOR_ENCODER_POSITION_FACTOR = SPROCKET_DIAMETER * Math.PI;

    private static final double KP = 0.13;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double KS = 0.0;
    private static final double KG = 0.0;
    private static final double KV = 0.0;
    private static final double KA = 0.0;
    private static final double EXPO_KV = 0.0;
    private static final double EXPO_KA = 0.0;

    private static final double MAX_VELOCITY = 0;
    private static final double MAX_ACCELERATION = 0;
    private static final double JERK_TARGET = 0;

    private static final TalonFXConfiguration BASE_CONFIG = new TalonFXConfiguration();
    static {
        var slot0Configs = BASE_CONFIG.Slot0;
        slot0Configs.kP = KP;
        slot0Configs.kI = KI;
        slot0Configs.kD = KD;
        slot0Configs.kS = KS;
        slot0Configs.kG = KG;
        slot0Configs.kV = KV;
        slot0Configs.kA = KA;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = BASE_CONFIG.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MAX_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK_TARGET;
        motionMagicConfigs.MotionMagicExpo_kV = EXPO_KV;
        motionMagicConfigs.MotionMagicExpo_kA = EXPO_KA;

        var softLimitConfigs = BASE_CONFIG.SoftwareLimitSwitch;
        softLimitConfigs.ForwardSoftLimitEnable = true;
        softLimitConfigs.ForwardSoftLimitThreshold = metersToMotorPosition(ELEVATOR_TOP - 0.1);
        softLimitConfigs.ReverseSoftLimitEnable = true;
        softLimitConfigs.ReverseSoftLimitThreshold = metersToMotorPosition(ELEVATOR_BOTTOM + 0.1);
    }
    //#endregion

    private final TalonFX leader;
    private final TalonFX follower;

    private final TalonFXConfiguration config;

    private final MotionMagicExpoVoltage motionMagicExpoVoltageRequest;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltageRequest;
    private final VoltageOut voltageRequest;

    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Voltage> leaderVoltage;
    private final StatusSignal<Current> leaderCurrent;
    private final StatusSignal<Temperature> leaderTemperature;
    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerCurrent;
    private final StatusSignal<Temperature> followerTemperature;

    private double targetHeightMeters = 0.0;

    public ElevatorIOMotionMagic(int leftMotorID, int rightMotorID) {
        this(leftMotorID, rightMotorID, false);
    }

    public ElevatorIOMotionMagic(int leftMotorID, int rightMotorID, boolean invertRight) {
        this.leader = new TalonFX(leftMotorID);
        this.follower = new TalonFX(rightMotorID);

        this.config = BASE_CONFIG;
        this.leader.getConfigurator().apply(config);
        this.follower.getConfigurator().apply(config);

        this.follower.setControl(new Follower(leftMotorID, invertRight));

        this.leader.setNeutralMode(NeutralModeValue.Brake);
        this.follower.setNeutralMode(NeutralModeValue.Brake);
        this.leader.setPosition(0.0);
        this.follower.setPosition(0.0);

        this.motionMagicExpoVoltageRequest = new MotionMagicExpoVoltage(0);
        this.motionMagicVelocityVoltageRequest = new MotionMagicVelocityVoltage(0);
        this.voltageRequest = new VoltageOut(0);

        this.leaderPosition = leader.getPosition();
        this.leaderVelocity = leader.getVelocity();
        this.leaderVoltage = leader.getMotorVoltage();
        this.leaderCurrent = leader.getStatorCurrent();
        this.leaderTemperature = leader.getDeviceTemp();
        this.followerPosition = follower.getPosition();
        this.followerVelocity = follower.getVelocity();
        this.followerVoltage = follower.getMotorVoltage();
        this.followerCurrent = follower.getStatorCurrent();
        this.followerTemperature = follower.getDeviceTemp();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderVoltage,
            leaderCurrent,
            leaderTemperature,
            followerPosition,
            followerVelocity,
            followerVoltage,
            followerCurrent,
            followerTemperature
        );

        inputs.elevatorConnected = leader.isConnected() && follower.isConnected();
        inputs.heightMeters = motorPositionToMeters((leaderPosition.getValueAsDouble() + followerPosition.getValueAsDouble()) / 2);
        inputs.velocityMetersPerSecond = motorPositionToMeters((leaderVelocity.getValueAsDouble() + followerVelocity.getValueAsDouble()) / 2);
        inputs.heightSetpointMeters = targetHeightMeters;
        inputs.outOfBounds = false;
    };

    @Override
    public void applySetpointMeters(final double position) {
        targetHeightMeters = position;
        leader.setControl(motionMagicExpoVoltageRequest.withPosition(metersToMotorPosition(position)).withFeedForward(position));
    };

    @Override
    public void applyVelocityMetersPerSecond(final double velocity) {
        leader.setControl(motionMagicVelocityVoltageRequest.withVelocity(metersToMotorPosition(velocity)));
    };

    @Override
    public void applyDutyCycle(final double output) {
        leader.setControl(voltageRequest.withOutput(output * leader.getSupplyVoltage().getValueAsDouble()));
    };

    @Override
    public void applyIntakeVelocity(final double velocity) {

    };

    @Override
    public void applyIntakeDutyCycle(final double output) {
        
    };

    private static double motorPositionToMeters(double motorPosition) {
        return motorPosition * MOTOR_ENCODER_POSITION_FACTOR;
    }

    private static double metersToMotorPosition(double meters) {
        return meters / MOTOR_ENCODER_POSITION_FACTOR;
    }
}
