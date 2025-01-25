// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;


import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AlgaeIntakeConstants;

/** Add your docs here. */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final SparkMaxSim rollerMotorSim;
    private final SparkMaxSim angleMotorSim;

    private boolean rollerClosedLoop = false;
    private boolean angleClosedLoop = false;
    private final PIDController rollerController = new PIDController(
        AlgaeIntakeConstants.ROLLER_KP, 0, AlgaeIntakeConstants.ROLLER_KD);
    private final PIDController angleController = new PIDController(
        AlgaeIntakeConstants.ANGLE_KP, 0, AlgaeIntakeConstants.ANGLE_KD);
    private double rollerFFVolts = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double angleAppliedVolts = 0.0;

    public AlgaeIntakeIOSim() {
        this.rollerMotorSim = new SparkMaxSim(
            new SparkMax(AlgaeIntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless),
            DCMotor.getNeo550(1));
        this.angleMotorSim = new SparkMaxSim(
            new SparkMax(AlgaeIntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless),
            DCMotor.getNEO(1));

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        if (rollerClosedLoop) {
            rollerAppliedVolts =
                rollerFFVolts + rollerController.calculate(rollerMotorSim.getVelocity());
        } else {
            rollerController.reset();
        }

        if (angleClosedLoop) {
            angleAppliedVolts =
                angleController.calculate(angleMotorSim.getPosition());
        } else {
            angleController.reset();
        }

        rollerMotorSim.iterate(rollerController.getSetpoint(), rollerAppliedVolts, 0.005);
        angleMotorSim.iterate(angleController.getSetpoint(), angleAppliedVolts, 0.005);

        inputs.rollerConnected = true;
        inputs.rollerVelocity = rollerMotorSim.getVelocity();
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrent = Math.abs(rollerMotorSim.getMotorCurrent());

        inputs.angleConnected = true;
        inputs.angleVelocity = angleMotorSim.getVelocity();
        inputs.angleAppliedVolts = angleAppliedVolts;
        inputs.anglePosition = Rotation2d.fromRadians(angleMotorSim.getPosition());
        inputs.angleCurrent = Math.abs(angleMotorSim.getMotorCurrent());
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerFFVolts =
            AlgaeIntakeConstants.ROLLER_KS * Math.signum(velocity)
                + AlgaeIntakeConstants.ROLLER_KV * velocity;
        rollerClosedLoop = true;
        rollerController.setSetpoint(velocity);
    }

    @Override
    public void setRollerOpenLoop(double volts) {
        rollerAppliedVolts = volts;
        rollerClosedLoop = false;
    }

    @Override
    public void setAnglePosition(Rotation2d position) {
        angleClosedLoop = true;
        angleController.setSetpoint(position.getRadians());
    }

    @Override
    public void setAngleOpenLoop(double volts) {
        angleAppliedVolts = volts;
        angleClosedLoop = false;
    }

    @Override
    public AlgaeIntakeGains getGains() {
        return new AlgaeIntakeGains(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public void setGains(AlgaeIntakeGains gains) {}
}
