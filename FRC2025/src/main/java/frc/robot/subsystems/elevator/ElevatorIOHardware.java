// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.motor.TalonFXConfig;
import frc.robot.util.motor.TalonFXController;
import frc.robot.util.motor.TalonFXIO;

/** Add your docs here. */
public class ElevatorIOHardware implements ElevatorIO {
    private final TalonFXController leftMotor;
    private final TalonFXController rightMotor;

    public ElevatorIOHardware(TalonFXConfig leftMotor, TalonFXConfig rightMotor) {
        this.leftMotor = new TalonFXController(
            "Elevator",
            "LeftMotor",
            new TalonFXIO(leftMotor));
        this.rightMotor = new TalonFXController(
            "Elevator",
            "RightMotor",
            new TalonFXIO(rightMotor));
    }

    private double getHeightMeters(double left, double right) {
        double leftHeight = left * ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS;
        double rightHeight = right * ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS;
        return leftHeight > rightHeight ? rightHeight : leftHeight;
    }

    @Override
    public void periodic() {
        leftMotor.periodic();
        rightMotor.periodic();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorConnected = true;

        inputs.elevatorHeightMeters = getHeightMeters(leftMotor.getPosition(), rightMotor.getPosition());
        inputs.elevatorVelocityMetersPerSecond = getHeightMeters(leftMotor.getVelocity(), rightMotor.getVelocity());
    }

    @Override
    public void setHeight(double heightMeters) {
        leftMotor.setMotionMagicSetpoint(heightMeters / ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS);
        rightMotor.setMotionMagicSetpoint(heightMeters / ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond) {
        leftMotor.setVelocitySetpoint(velocityMetersPerSecond / ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS);
        rightMotor.setVelocitySetpoint(velocityMetersPerSecond / ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_METERS);
    }
}
