// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static frc.robot.util.MotorUtils.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AlgaeIntakeConstants.AngleConfig;
import frc.robot.Constants.AlgaeIntakeConstants.RollerConfig;
import frc.robot.SparkModuleConstants;

/** Add your docs here. */
public class AlgaeIntakeIOHardware implements AlgaeIntakeIO {
    protected final SparkMax rollerMotor;
    protected final SparkMax angleMotor;
    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkBaseConfig rollerConfig;
    private final SparkBaseConfig angleConfig;

    private AlgaeIntakeGains gains;

    private final SparkClosedLoopController rollerController;
    private final SparkClosedLoopController angleController;

    private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer angleConnectedDebounce = new Debouncer(0.5);

    public AlgaeIntakeIOHardware(RollerConfig rollerConfig, AngleConfig angleConfig) {
        rollerMotor = new SparkMax(rollerConfig.ID(), MotorType.kBrushless);
        angleMotor = new SparkMax(angleConfig.ID(), MotorType.kBrushless);

        rollerEncoder = rollerMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        rollerController = rollerMotor.getClosedLoopController();
        angleController = angleMotor.getClosedLoopController();

        this.rollerConfig = AlgaeIntakeConstants.ROLLER_BASE_CONFIG;
        this.rollerConfig.inverted(rollerConfig.invert());
        tryUntilOk(rollerMotor, 5, () ->
            rollerMotor.configure(this.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerMotor, 5, () -> rollerEncoder.setPosition(0.0));

        this.angleConfig = AlgaeIntakeConstants.ANGLE_BASE_CONFIG;
        this.angleConfig.inverted(angleConfig.invert());
        tryUntilOk(angleMotor, 5, () ->
            angleEncoder.setPosition(0.0));

        gains = new AlgaeIntakeGains(
            AlgaeIntakeConstants.ROLLER_KP,
            0.0,
            AlgaeIntakeConstants.ROLLER_KD,
            AlgaeIntakeConstants.ROLLER_KS,
            AlgaeIntakeConstants.ROLLER_KV,
            0.0
        );
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.rollerVelocity = value);
        ifOk(rollerMotor,
            new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
            (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
        ifOk(rollerMotor, rollerMotor::getOutputCurrent, (value) -> inputs.rollerCurrent = value);
        inputs.rollerConnected = rollerConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(angleMotor, angleEncoder::getVelocity, (value) -> inputs.angleVelocity = value);
        ifOk(angleMotor,
            angleEncoder::getPosition,
            (value) -> inputs.anglePosition = new Rotation2d(value));
        ifOk(angleMotor,
            new DoubleSupplier[] {angleMotor::getAppliedOutput, angleMotor::getBusVoltage},
            (values) -> inputs.angleAppliedVolts = values[0] * values[1]);
        ifOk(angleMotor, angleMotor::getOutputCurrent, (value) -> inputs.angleCurrent = value);
        inputs.angleConnected = angleConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setRollerVelocity(double velocity) {
        double ffVolts =
            AlgaeIntakeConstants.ROLLER_KS * Math.signum(velocity)
            + AlgaeIntakeConstants.ROLLER_KV * velocity;
        rollerController.setReference(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setRollerOpenLoop(double volts) {
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void setAnglePosition(Rotation2d rotation) {
        double setPoint =
            MathUtil.inputModulus(
                rotation.getRadians(),
                SparkModuleConstants.turnPIDMinInput,
                SparkModuleConstants.turnPIDMaxInput);
        angleController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public void setAngleOpenLoop(double volts) {
        angleMotor.setVoltage(volts);
    }

    @Override
    public AlgaeIntakeGains getGains() {
        return gains;
    }

    @Override
    public void setGains(AlgaeIntakeGains gains) {
        this.gains = gains;
        
        tryUntilOk(
            rollerMotor,
            5,
            () -> rollerMotor.configure(
                rollerConfig.apply(
                    new ClosedLoopConfig().pidf(gains.rollerKP(), gains.rollerKI(), gains.rollerKD(), gains.rollerKV())),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters));
    }
}
