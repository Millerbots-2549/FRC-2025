// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.motor.MotorIOTalonFX;
import frc.robot.util.motor.MotorIO.MotorIOInputs;

/** Add your docs here. */
public class ElevatorIOHardware implements ElevatorIO {
    private final MotorIOTalonFX leftMotor;
    private final MotorIOTalonFX rightMotor;

    private final MotorIOInputs leftMotorInputs = new MotorIOInputs();
    private final MotorIOInputs rightMotorInputs = new MotorIOInputs();

    private double elevatorSetpointHeightMeters;

    private PIDController heightPID = new PIDController(0.03, 0.0, 0.0);

    public ElevatorIOHardware(MotorIOTalonFX leftMotor, MotorIOTalonFX rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        elevatorSetpointHeightMeters = 0.0;
    }

    private double motorPositionToHeight(double position) {
        return position * ElevatorConstants.MOTOR_TO_HEIGHT_RATIO;
    }

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

        SmartDashboard.putNumber("Elevator Left Motor Position", leftMotorInputs.position);
        SmartDashboard.putNumber("Elevator Right Motor Position", rightMotorInputs.position);

        double output = heightPID.calculate(inputs.heightMeters, elevatorSetpointHeightMeters);
        SmartDashboard.putNumber("Wanted Output", output);
        output = MathUtil.clamp(output + 0.015, -0.19, 0.27);
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
}
