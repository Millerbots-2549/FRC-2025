// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.descorer;

import static frc.robot.Constants.DescorerConstants.*;
import static frc.robot.util.MotorUtils.ifOk;
import static frc.robot.util.MotorUtils.sparkStickyFault;
import static frc.robot.util.MotorUtils.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.motor.ArmFeedForward;

/** Add your docs here. */
public class DescorerIOHardware implements DescorerIO {
    /** This is the spark max used to turn the roller */
    protected final SparkMax wristMotor;
    private final RelativeEncoder wristEncoder;

    private final SparkBaseConfig wristConfig;

    private final Constraints wristConstraints = 
        new Constraints(WRIST_MAX_VELOCITY, WRIST_MAX_ACCEL);
    private final ArmFeedForward wristFeedForward =
        new ArmFeedForward(WRIST_KS, WRIST_KG, WRIST_KV, WRIST_KA);
    private final ProfiledPIDController wristPositionController = 
        new ProfiledPIDController(WRIST_KP, WRIST_KI, WRIST_KD, wristConstraints);

    private Rotation2d setpoint;

    private final Debouncer wristConnectedDebounce = new Debouncer(0.5);

    public DescorerIOHardware(SparkMaxConfig wristConfig) {
        this.wristMotor = new SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        this.wristEncoder = wristMotor.getEncoder();

        this.wristConfig = wristConfig;
        tryUntilOk(wristMotor, 5, () ->
            wristMotor.configure(this.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(wristMotor, 5, () -> wristEncoder.setPosition(0));

        wristPositionController.enableContinuousInput(WRIST_PID_MIN_INPUT, WRIST_PID_MAX_INPUT);
        wristPositionController.setTolerance(0.1, 0.1);
    }

    @Override
    public void updateInputs(DescorerIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(wristMotor, wristEncoder::getVelocity, (value) -> inputs.wristVelocity = value);
        ifOk(wristMotor,
            wristEncoder::getPosition,
            (value) -> inputs.wristPosition = new Rotation2d(value));
        ifOk(wristMotor,
            new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
            (values) -> inputs.wristAppliedVolts = values[0] * values[1]);
        ifOk(wristMotor, wristMotor::getOutputCurrent, (value) -> inputs.wristCurrent = value);
        inputs.wristConnected = wristConnectedDebounce.calculate(!sparkStickyFault);
    }

    public void periodic() {
        double absolutePosition =
            MathUtil.inputModulus(
                wristEncoder.getPosition(),
                WRIST_PID_MIN_INPUT,
                WRIST_PID_MAX_INPUT
            );
        double setpointPosition =
            MathUtil.inputModulus(
                -setpoint.getRadians(),
                WRIST_PID_MIN_INPUT,
                WRIST_PID_MAX_INPUT
            );
        double output = wristPositionController.calculate(absolutePosition, setpointPosition);
        output += wristFeedForward.calculate(setpointPosition, 0.0);
    }

    @Override
    public void applyOpenLoop(double output) {};

    @Override
    public void applyVelocity(double velocity) {};

    @Override
    public void applySetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
    };

    @Override
    public Rotation2d getCurrentSetpoint() {
        return setpoint;
    }
}
