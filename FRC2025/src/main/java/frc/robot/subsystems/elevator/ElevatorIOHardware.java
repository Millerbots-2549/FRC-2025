// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.Orchestra;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.motor.MotorIO.MotorIOInputs;
import frc.robot.util.motor.MotorIOTalonFX;

import static frc.robot.util.MotorUtils.*;

/** Add your docs here. */
public class ElevatorIOHardware implements ElevatorIO {
    private final MotorIOTalonFX leftMotor;
    private final MotorIOTalonFX rightMotor;

    private final MotorIOInputs leftMotorInputs = new MotorIOInputs();
    private final MotorIOInputs rightMotorInputs = new MotorIOInputs();

    private final SparkBaseConfig intakeConfig;
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    private PIDController intakeSpeedPID = new PIDController(0.015, 0, 0);

    private double elevatorSetpointHeightMeters;

    private Orchestra orchestra = new Orchestra();

    private ProfiledPIDController heightPID = new ProfiledPIDController(0.035, 0.0, 0.0, new Constraints(40, 55));

    public ElevatorIOHardware(MotorIOTalonFX leftMotor, MotorIOTalonFX rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.intakeMotor = new SparkMax(ElevatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.intakeEncoder = intakeMotor.getEncoder();

        this.intakeConfig = ElevatorConstants.INTAKE_BASE_CONFIG;
        tryUntilOk(intakeMotor, 5, () ->
            intakeMotor.configure(this.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(intakeMotor, 5, () -> intakeEncoder.setPosition(0.0));

        elevatorSetpointHeightMeters = 0.0;

        orchestra.addInstrument(leftMotor.motor);
        orchestra.addInstrument(rightMotor.motor);

        var status = orchestra.loadMusic("music/badapple.chrp");

        if (status.isError()) {
            System.out.println(status.toString());
        }
    }

    private double motorPositionToHeight(double position) {
        return position * ElevatorConstants.MOTOR_TO_HEIGHT_RATIO;
    }

    @SuppressWarnings("unused")
    private double heightToMotorPosition(double height) {
        return height / ElevatorConstants.MOTOR_TO_HEIGHT_RATIO;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        leftMotor.updateInputs(leftMotorInputs);
        rightMotor.updateInputs(rightMotorInputs);

        inputs.elevatorConnected = leftMotorInputs.connected && rightMotorInputs.connected;
        inputs.heightMeters = (motorPositionToHeight(leftMotorInputs.position)
            + motorPositionToHeight(rightMotorInputs.position)) / 2;
        inputs.velocityMetersPerSecond = (motorPositionToHeight(leftMotorInputs.velocity)
            + motorPositionToHeight(rightMotorInputs.velocity)) / 2;
            inputs.heightSetpointMeters = elevatorSetpointHeightMeters;
        boolean leftOverpowered = leftMotorInputs.statorCurrentAmps >= ElevatorConstants.CURRENT_UPPER_BOUND;
        boolean rightOverpowered = rightMotorInputs.statorCurrentAmps >= ElevatorConstants.CURRENT_UPPER_BOUND;
        boolean goingDown = inputs.velocityMetersPerSecond < 0;
        inputs.outOfBounds = (leftOverpowered || rightOverpowered) && goingDown;

        double g_offset = elevatorSetpointHeightMeters > 20.0 ? ElevatorConstants.ELEVATOR_KG : -ElevatorConstants.ELEVATOR_KG;
        double output = heightPID.calculate(inputs.heightMeters, elevatorSetpointHeightMeters + 0.4);
        output = MathUtil.clamp(output + g_offset, -0.21, 0.29);
        leftMotor.applyDutyCycle(output);
        rightMotor.applyDutyCycle(output);
    }

    @Override
    public void applySetpointMeters(double position) {
        elevatorSetpointHeightMeters = position;
        //leftMotor.applyProfileSetpoint(heightToMotorPosition(position));
        //rightMotor.applyProfileSetpoint(heightToMotorPosition(position));
    }

    @Override
    public void applyVelocityMetersPerSecond(double velocity) {
        leftMotor.applyVelocity(motorPositionToHeight(velocity));
        rightMotor.applyVelocity(motorPositionToHeight(velocity));
    }

    @Override
    public void applyDutyCycle(double output) {
        double newOutput = output;
        leftMotor.applyDutyCycle(newOutput);
        rightMotor.applyDutyCycle(newOutput);
    }

    @Override
    public void applyIntakeVelocity(double velocity) {
        double output = intakeSpeedPID.calculate(intakeEncoder.getVelocity(), velocity);
        output = MathUtil.clamp(output, -1.0, 1.0);
        intakeMotor.set(output);
    }

    @Override
    public void applyIntakeDutyCycle(double output) {
        if (Math.abs(output) < 0.001) {
            output = 0;
        }
        intakeMotor.set(output);
    }

    @Override
    public void playMusic() {
        orchestra.play();
    }

    @Override
    public void stopMusic() {
        orchestra.stop();
    }
}
