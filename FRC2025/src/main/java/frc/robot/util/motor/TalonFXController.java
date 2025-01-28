// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXController {
    private final String parent;
    private final String name;

    private final TalonFXIO io;
    private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

    public TalonFXController(String parent, String name, TalonFXIO io) {
        this.parent = parent;
        this.name = name;
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(parent + "/" + name, inputs);
    }

    public void setVoltage(double voltage) {
        io.setOpenLoopDutyCycle(voltage / 12.0);
    }

    public void setPositionSetpoint(double position) {
        io.setPositionSetpoint(position);
    }

    public void setMotionMagicSetpoint(double position) {
        io.setMotionMagicSetpoint(position);
    }

    public void setNeutralMode(NeutralModeValue mode) {
        io.setNeutralMode(mode);
    }

    public void setEnableSoftwareLimits(boolean forward, boolean reverse) {
        io.setEnableSoftwareLimits(forward, reverse);
    }

    public void setVelocitySetpoint(double velocity) {
        io.setVelocitySetpoint(velocity);
    }

    public void zeroPosition() {
        io.zeroPosition();
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getCurrentStatorAmps() {
        return inputs.currentStatorAmps;
    }

    public double getCurrentSupplyAmps() {
        return inputs.currentSupplyAmps;
    }

    public double getVoltage() {
        return inputs.appliedVolts;
    }
}
