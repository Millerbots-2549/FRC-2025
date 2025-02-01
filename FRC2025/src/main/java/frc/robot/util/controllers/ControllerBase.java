// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controllers;

/** Add your docs here. */
public interface ControllerBase {
    default void setReference(double setpoint) {
        setReference(setpoint, 0.0);
    }

    public void setReference(double setpoint, double feedforward);

    public void enableContinuousInput(double minimumInput, double maximumInput);

    public void disableContinuousInput();

    public boolean isContinuousInputEnabled();

    public boolean bindToLogger(String path);
}
