// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static frc.robot.util.MotorUtils.ifOk;
import static frc.robot.util.MotorUtils.sparkStickyFault;
import static frc.robot.util.MotorUtils.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AlgaeIntakeConstants.AngleConfig;
import frc.robot.Constants.AlgaeIntakeConstants.RollerConfig;

/**
 * This is an implementation of the {@link AlgaeIntakeIO} interface which uses
 * two {@link SparkMax} motor controllers, and a NEO 550 for the roller, and a
 * NEO 1.1 for the angle motor.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class AlgaeIntakeIOHardware implements AlgaeIntakeIO {
    /** This is the spark max used to turn the roller */
    protected final SparkMax rollerMotor;
    /** This is the spark max used to move the algae intake up and down. */
    protected final SparkMax angleMotor;
    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkBaseConfig rollerConfig;
    private final SparkBaseConfig angleConfig;

    /** The {@link SparkClosedLoopController} for the roller motor. */
    @SuppressWarnings("unused")
    private final SparkClosedLoopController rollerClosedLoopController;
    /**
     * The {@link PIDController} for the angle motor. We have to use a generic PIDController
     * Instead of a {@link SparkClosedLoopController} because otherwise it doesn't work for
     * some reason
     */
    private final ProfiledPIDController angleController;
    private final PIDController evilAngleController;

    private double angleSetpoint;

    private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer angleConnectedDebounce = new Debouncer(0.5);

    public AlgaeIntakeIOHardware(RollerConfig rollerConfig, AngleConfig angleConfig) {
        // Creating the objects for controlling the hardware
        rollerMotor = new SparkMax(rollerConfig.ID(), MotorType.kBrushless);
        angleMotor = new SparkMax(angleConfig.ID(), MotorType.kBrushless);

        rollerEncoder = rollerMotor.getEncoder(); // Relative encoder
        angleEncoder = angleMotor.getEncoder(); // Relative encode=

        rollerClosedLoopController = rollerMotor.getClosedLoopController();
        angleController = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10)); // This is set up later in the constructor
        evilAngleController = new PIDController(0, 0, 0); // This is set up later in the constructor

        // Configurating the roller motor
        this.rollerConfig = AlgaeIntakeConstants.ROLLER_BASE_CONFIG;
        this.rollerConfig.inverted(rollerConfig.invert());
        tryUntilOk(rollerMotor, 5, () ->
            rollerMotor.configure(this.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerMotor, 5, () -> rollerEncoder.setPosition(0.0));

        // Configurating the angle motor
        this.angleConfig = AlgaeIntakeConstants.ANGLE_BASE_CONFIG;
        this.angleConfig.inverted(angleConfig.invert());
        this.angleMotor.configure(this.angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tryUntilOk(angleMotor, 5, () ->
            angleEncoder.setPosition(0.0));

        // Setting up the angle motor PID
        angleController.setPID(AlgaeIntakeConstants.ANGLE_KP, 0.0, AlgaeIntakeConstants.ANGLE_KD);
        angleController.enableContinuousInput(AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT, AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT);
        angleController.setTolerance(0.1, 0.1);
        evilAngleController.setPID(0.17, 0.0, 0.0);
        evilAngleController.enableContinuousInput(AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT, AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT);
        evilAngleController.setTolerance(0.1, 0.1);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        // Updating inputs for the roller motor
        sparkStickyFault = false;
        ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.rollerVelocity = value);
        ifOk(rollerMotor,
            new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
            (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
        ifOk(rollerMotor, rollerMotor::getOutputCurrent, (value) -> inputs.rollerCurrent = value);
        inputs.rollerConnected = rollerConnectedDebounce.calculate(!sparkStickyFault);

        // Updating inputs for the angle motor
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

        inputs.currentAngle = MathUtil.inputModulus(
            angleEncoder.getPosition(),
            AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
            AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT);
        inputs.currentSetpoint = angleSetpoint;
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMotor.set(velocity);
    }

    @Override
    public void setRollerOpenLoop(double volts) {
        rollerMotor.setVoltage(-volts);
    }

    @Override
    public void setAnglePosition(Rotation2d rotation) {
        // Sets the measurement to the correct range
        double absolutePosition = 
            MathUtil.inputModulus(
                angleEncoder.getPosition(),
                AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
                AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
            );
        // This converts the rotation into radians in the correct range
        double setPoint =
            MathUtil.inputModulus(
                rotation.getRadians(),
                AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
                AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
            );
        angleController.setTolerance(0.1, 0.1);
        double output = angleController.calculate(absolutePosition, setPoint);
        output -= output < 0 ?  Math.sin(absolutePosition) * AlgaeIntakeConstants.ANGLE_KG
                : Math.sin(absolutePosition) * AlgaeIntakeConstants.ANGLE_KG;
        angleMotor.set(MathUtil.clamp(output, -1.0, 1.0));

        angleSetpoint = setPoint;
    }

    @Override
    public void setAnglePosition(Rotation2d rotation, boolean resist) {
        // Sets the measurement to the correct range
        double absolutePosition = 
            MathUtil.inputModulus(
                angleEncoder.getPosition(),
                AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
                AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
            );
        // This converts the rotation into radians in the correct range
        double setPoint =
            MathUtil.inputModulus(
                -rotation.getRadians(),
                AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
                AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
            );
        evilAngleController.setTolerance(0.1, 0.1);
        double output = angleController.calculate(absolutePosition, setPoint);
        output -= Math.sin(absolutePosition) * AlgaeIntakeConstants.ANGLE_KG;
        angleMotor.set(MathUtil.clamp(output, -0.75, 0.75));
    }

    @Override
    public void setAngleOpenLoop(double volts) {
        angleMotor.setVoltage(volts);
    }
}
