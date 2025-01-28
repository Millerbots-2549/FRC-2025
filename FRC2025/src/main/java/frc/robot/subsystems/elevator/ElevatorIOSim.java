// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
    private double targetHeight = 0.0;
    private double height = 0.0;
    private double velocity = 0.0;
    private PIDController pid = new PIDController(0.05, 0.0, 0.0);

    public ElevatorIOSim() {
        pid.setP(0.05);
        pid.setI(0.0);
        pid.setD(0.0);
    }

    @Override
    public void periodic() {
        velocity = pid.calculate(height, targetHeight);
        if (velocity > 0.01) {
            velocity = 0.01;
        }
        if (velocity < -0.01) {
            velocity = -0.01;
        }
        height += velocity;
        Logger.recordOutput("Elevator/Height", height);
        Logger.recordOutput("Elevator/Velocity", velocity);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorConnected = true;

        inputs.elevatorHeightMeters = height;
        inputs.elevatorVelocityMetersPerSecond = velocity;
    }

    @Override
    public void setHeight(double height) {
        this.targetHeight = height;
    }

    @Override
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }
}
