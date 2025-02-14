// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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
    }

    @Override
    public void applySetpointMeters(double position) {
        elevatorSetpointHeightMeters = position;
        leftMotor.applyProfileSetpoint(heightToMotorPosition(position));
        rightMotor.applyProfileSetpoint(heightToMotorPosition(position));
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
