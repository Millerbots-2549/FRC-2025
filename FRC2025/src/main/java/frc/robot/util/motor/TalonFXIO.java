// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static frc.robot.util.MotorUtils.*;

/** Add your docs here. */
public class TalonFXIO {
    @AutoLog
    public static class TalonFXIOInputs {
        public double velocity = 0.0;
        public double position = 0.0;
        public double appliedVolts = 0.0;
        public double currentStatorAmps = 0.0;
        public double currentSupplyAmps = 0.0;
    }

    protected final TalonFX talon;
    protected final TalonFXConfig config;

    protected final DutyCycleOut dutyCycleController = new DutyCycleOut(0);
    private final VelocityVoltage velocityVoltageController = new VelocityVoltage(0);
    private final PositionVoltage positionVoltageController = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicController = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;

    private BaseStatusSignal[] statusSignals = new BaseStatusSignal[] {};

    public TalonFXIO(TalonFXConfig config) {
        this.config = config;
        talon = new TalonFX(config.canID);

        if (Constants.currentMode == Constants.Mode.SIM) {
            this.config.config.CurrentLimits = new CurrentLimitsConfigs();
            this.config.config.ClosedLoopRamps = new ClosedLoopRampsConfigs();
            this.config.config.OpenLoopRamps = new OpenLoopRampsConfigs();
        }

        tryUntilOk(10, () -> talon.getConfigurator().apply(config.config));

        positionSignal = talon.getPosition();
        velocitySignal = talon.getVelocity();
        voltageSignal = talon.getMotorVoltage();
        currentStatorSignal = talon.getStatorCurrent();
        currentSupplySignal = talon.getSupplyCurrent();

        statusSignals = new BaseStatusSignal[] {
            positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
        };

        tryUntilOk(10, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, statusSignals));
        tryUntilOk(10, () -> talon.optimizeBusUtilization());
    }

    private double rotorToUnits(double rotor) {
        return rotor * config.rotorRatio;
    }

    private double unitsToRotor(double units) {
        return units / config.rotorRatio;
    }

    private double clampPosition(double units) {
        return unitsToRotor(MathUtil.clamp(units, config.MIN_POSITION, config.MAX_POSITION));
    }

    public void updateInputs(TalonFXIOInputs inputs) {
        BaseStatusSignal.refreshAll(statusSignals);

        inputs.position = rotorToUnits(positionSignal.getValueAsDouble());
        inputs.velocity = rotorToUnits(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
        inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    }

    public void setOpenLoopDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleController.withOutput(dutyCycle));
    }

    public void setPositionSetpoint(double position) {
        talon.setControl(positionVoltageController.withPosition(clampPosition(position)));
    }

    public void setMotionMagicSetpoint(double position) {
        talon.setControl(motionMagicController.withPosition(clampPosition(position)));
    }

    public void setNeutralMode(NeutralModeValue mode) {
        config.config.MotorOutput.NeutralMode = mode;
        tryUntilOk(10, () -> talon.getConfigurator().apply(config.config));
    }

    public void setEnableSoftwareLimits(boolean forward, boolean reverse) {
        config.config.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
        config.config.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
        tryUntilOk(10, () -> talon.getConfigurator().apply(config.config));
    }

    public void setVelocitySetpoint(double unitsPerSecond) {
        talon.setControl(velocityVoltageController.withVelocity(unitsToRotor(unitsPerSecond)));
    }

    public void zeroPosition() {
        setPosition(0.0);
    }

    public void setPosition(double position) {
        talon.setPosition(position / config.rotorRatio);
    }
}
