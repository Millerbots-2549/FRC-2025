// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class TalonFXIOSim extends TalonFXIO {
    protected DCMotorSim motorSim;
    private Notifier notifier = null;
    private double lastUpdateTime = 0.0;

    public TalonFXIOSim(TalonFXConfig config) {
        super(config);

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                config.intertia, config.rotorRatio),
            DCMotor.getKrakenX60Foc(1));
        
        talon.getSimState().Orientation = (config.config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
        
        notifier = new Notifier(() -> {
            updateSimState();
        });
        notifier.startPeriodic(0.005);
    }

    protected double addFriction(double motorVolts, double friction) {
        if (Math.abs(motorVolts) < friction) {
            motorVolts = 0.0;
        } else if (motorVolts > 0.0) {
            motorVolts -= friction;
        } else {
            motorVolts += friction;
        }
        return motorVolts;
    }

    protected void updateSimState() {
        TalonFXSimState simState = talon.getSimState();
        double simVolts = addFriction(simState.getMotorVoltage(), 0.25);

        motorSim.setInput(simVolts);
        Logger.recordOutput(config.name + "/Sim/SimVolts", simVolts);

        double timestamp = Logger.getTimestamp() * 1.0E-6;
        motorSim.update(timestamp - lastUpdateTime);
        lastUpdateTime = timestamp;

        double simPositionRads = motorSim.getAngularPositionRad();
        Logger.recordOutput(config.name + "/Sim/PositionRads", simPositionRads);

        double rotorPosition = Units.radiansToRotations(simPositionRads) / config.rotorRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput(config.name + "/Sim/RotorPosition", rotorPosition);

        double rotorVel = Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec()) / config.rotorRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput(config.name + "/Sim/RotorVelocity", motorSim.getAngularVelocityRadPerSec());
    }
}
