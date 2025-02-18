// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.AlgaeIntakeConstants;

/**
 * This is an implementation of the {@link AlgaeIntakeIO} interface which uses
 * two {@link DCMotorSim} objects to simulate how the intake <i>should</i>
 * move. This class uses preset gains in {@link AlgaeIntakeConstants}
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final DCMotorSim rollerMotorSim;
    private final DCMotorSim angleMotorSim;

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
        this.rollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1),
            1.0, AlgaeIntakeConstants.ROLLER_GEAR_RATIO),
            DCMotor.getNeo550(1));
        this.angleMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1),
            0.0004, AlgaeIntakeConstants.ANGLE_GEAR_RATIO),
            DCMotor.getNEO(1));

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        if (rollerClosedLoop) {
            rollerAppliedVolts =
                rollerFFVolts + rollerController.calculate(rollerMotorSim.getAngularVelocityRadPerSec());
        } else {
            rollerController.reset();
        }

        if (angleClosedLoop) {
            angleAppliedVolts =
                angleController.calculate(angleMotorSim.getAngularPositionRad());
        } else {
            angleController.reset();
        }

        // Updates the simulation with time
        rollerMotorSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
        angleMotorSim.setInputVoltage(MathUtil.clamp(angleAppliedVolts, -12.0, 12.0));
        rollerMotorSim.update(0.02);
        angleMotorSim.update(0.02);

        inputs.rollerConnected = true;
        inputs.rollerVelocity = rollerMotorSim.getAngularVelocityRadPerSec();
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrent = Math.abs(rollerMotorSim.getCurrentDrawAmps());

        inputs.angleConnected = true;
        inputs.angleVelocity = angleMotorSim.getAngularVelocityRadPerSec();
        inputs.angleAppliedVolts = angleAppliedVolts;
        inputs.anglePosition = Rotation2d.fromRadians(angleMotorSim.getAngularPositionRad());
        inputs.angleCurrent = Math.abs(angleMotorSim.getCurrentDrawAmps());
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerClosedLoop = true;
        rollerFFVolts = 0.0;
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
}
