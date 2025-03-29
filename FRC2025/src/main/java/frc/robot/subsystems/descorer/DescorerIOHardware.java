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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DescorerIOHardware implements DescorerIO {
    protected final SparkMax wristMotor;
    protected final SparkMax rollerMotor;
    private final RelativeEncoder wristEncoder;
    private final RelativeEncoder rollerEncoder;

    private final SparkBaseConfig wristConfig;
    private final SparkBaseConfig rollerConfig;

    private boolean openLoop = false;

    private final Constraints wristConstraints = 
        new Constraints(WRIST_MAX_VELOCITY, WRIST_MAX_ACCEL);
    private final ProfiledPIDController wristPositionController = 
        new ProfiledPIDController(WRIST_KP, WRIST_KI, WRIST_KD, wristConstraints);
    
    private final Constraints rollerConstraints =
        new Constraints(ROLLER_MAX_VELOCITY, ROLLER_MAX_ACCEL);
    private final ProfiledPIDController rollerVelocityController =
        new ProfiledPIDController(ROLLER_KP, ROLLER_KI, WRIST_KD, rollerConstraints);

    private Rotation2d setpoint = DESCORER_OFF_POSITION;
    private double velocitySetpoint = 0.0;
    private Timer velocityTimer = new Timer();

    private final Debouncer wristConnectedDebounce = new Debouncer(0.5);

    public DescorerIOHardware(SparkBaseConfig wristConfig, SparkBaseConfig rollerConfig) {
        this.wristMotor = new SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
        this.rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

        this.wristEncoder = wristMotor.getEncoder();
        this.rollerEncoder = rollerMotor.getEncoder();

        this.wristConfig = wristConfig;
        wristConfig.idleMode(IdleMode.kCoast);
        tryUntilOk(wristMotor, 5, () ->
            wristMotor.configure(this.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(wristMotor, 5, () -> wristEncoder.setPosition(0));

        this.rollerConfig = rollerConfig;
        tryUntilOk(rollerMotor, 5, () ->
            rollerMotor.configure(this.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerMotor, 5, () -> rollerEncoder.setPosition(0));

        //wristPositionController.enableContinuousInput(WRIST_PID_MIN_INPUT, WRIST_PID_MAX_INPUT);
        wristPositionController.setTolerance(0.1, 0.1);

        rollerVelocityController.setTolerance(0.1);

        velocityTimer.start();
    }

    @Override
    public void updateInputs(DescorerIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(wristMotor, wristEncoder::getVelocity, (value) -> inputs.wristVelocityRadPerSec = value);
        ifOk(wristMotor,
            wristEncoder::getPosition,
            (value) -> inputs.wristPosition = Rotation2d.fromRadians(value));
        ifOk(wristMotor,
            new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
            (values) -> inputs.wristAppliedVolts = values[0] * values[1]);
        ifOk(wristMotor, wristMotor::getOutputCurrent, (value) -> inputs.wristCurrent = value);
        inputs.wristConnected = wristConnectedDebounce.calculate(!sparkStickyFault);
    }

    public void periodic() {
        /* 
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
            */

        double absolutePosition = wristEncoder.getPosition();
        double setpointPosition = setpoint.getRadians();

        SmartDashboard.putNumber("Desc Absolute Position", absolutePosition);
        SmartDashboard.putNumber("Desc Setpoint Position", setpointPosition);

        wristPositionController.setTolerance(0.001, 0.001);

        double output = wristPositionController.calculate(absolutePosition, setpointPosition);
        output += WRIST_KG * Math.cos(absolutePosition);
        output = output > 0 ? output * 0.5 : output;
        if(!openLoop) wristMotor.set(MathUtil.clamp(output, -0.9, 0.9));

        output = velocitySetpoint * (MathUtil.clamp(velocityTimer.get() * 3, 0.0, 1.0));
        rollerMotor.set(output);

        SmartDashboard.putBoolean("Open Loop", openLoop);
    }

    @Override
    public void applyWristDutyCycle(double output) {
        wristMotor.set(output);
        openLoop = true;
    };

    @Override
    public void applyWristSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
        this.wristPositionController.setPID(WRIST_KP, WRIST_KI, WRIST_KD);
        openLoop = false;
    };

    @Override
    public void applyWristSetpointStrong(Rotation2d setpoint) {
        this.setpoint = setpoint;
        this.wristPositionController.setPID(0.6, WRIST_KI, WRIST_KD);
        openLoop = false;
    };

    @Override
    public void applyRollerDutyCycle(double output) {
        if(velocitySetpoint != output) {
            velocityTimer.reset();
        }
        velocitySetpoint = output;
    }

    @Override
    public void applyRollerVelocity(double velocity) {
        this.velocitySetpoint = velocity;
    }

    @Override
    public Rotation2d getCurrentSetpoint() {
        return setpoint;
    }

    public Rotation2d getCurrentAbsolutePosition() {
        return Rotation2d.fromRotations(wristEncoder.getPosition());
    }
}
