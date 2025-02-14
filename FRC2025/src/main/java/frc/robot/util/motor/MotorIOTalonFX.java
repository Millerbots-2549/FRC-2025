// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import static frc.robot.util.MotorUtils.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class MotorIOTalonFX implements MotorIO {
    public final TalonFX motor;
    private final TalonFXConfiguration config;

    private final int ID;

    private final DutyCycleOut dutyCycleController = new DutyCycleOut(0);
    private final VelocityVoltage velocityVoltageController = new VelocityVoltage(0);
    private final PositionVoltage positionVoltageController = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicController = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private BaseStatusSignal[] statusSignals = new BaseStatusSignal[] {};

    public MotorIOTalonFX(int ID, TalonFXConfiguration config) {
        this.ID = ID;
        this.config = config;
        this.motor = new TalonFX(ID);

        tryUntilOk(10, () -> motor.getConfigurator().apply(config));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        currentStatorSignal = motor.getStatorCurrent();
        currentSupplySignal = motor.getSupplyCurrent();
        temperatureSignal = motor.getDeviceTemp();

        statusSignals = new BaseStatusSignal[] {
            positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
        };

        tryUntilOk(10, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, statusSignals));
        tryUntilOk(10, () -> motor.optimizeBusUtilization());

        zeroPosition();
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = motor.isConnected();

        BaseStatusSignal.refreshAll(statusSignals);

        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.position = positionSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();
        inputs.statorCurrentAmps = currentStatorSignal.getValueAsDouble();
        inputs.supplyCurrentAmps = currentSupplySignal.getValueAsDouble();
        inputs.motorTemperature = temperatureSignal.getValueAsDouble();
    }

    @Override
    public void applyVelocity(double velocity) {
        motor.setControl(velocityVoltageController.withVelocity(velocity));
    }

    @Override
    public void applyVoltage(double voltage) {
        motor.setControl(dutyCycleController.withOutput(
            voltage / motor.getSupplyVoltage().getValueAsDouble()));
    }

    @Override
    public void applyDutyCycle(double output) {
        motor.setControl(dutyCycleController.withOutput(output));
    }

    @Override
    public void applyProfileSetpoint(double setpoint) {
        motor.setControl(motionMagicController.withPosition(setpoint));
    }

    @Override
    public void applySetpoint(double setpoint) {
        motor.setControl(positionVoltageController.withPosition(setpoint));
    }

    @Override
    public void setNeutralMode(MotorNeutralMode neutralMode) {
        config.MotorOutput.NeutralMode = neutralMode == MotorNeutralMode.BRAKE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        tryUntilOk(10, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void setSoftLimits(boolean forward, boolean reverse) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
        tryUntilOk(10, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void zeroPosition() {
        motor.setPosition(0);
    }

    @Override
    public void setInternalPosition(double position) {
        motor.setPosition(position);
    }
}
