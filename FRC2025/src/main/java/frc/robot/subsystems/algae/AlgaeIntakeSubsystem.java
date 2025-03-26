// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.util.DashboardPublisher;
import frc.robot.RobotContainer;

/**
 * This is the subsystem for the algae intake. Can be simulated by using the {@link AlgaeIntakeIOSim} class
 * instead of {@link AlgaeIntakeIOHardware AlgaeIntakeIOHardware.}
 * 
 * <p>This class is <b>not</b> a singleton and needs to have an instance created in {@link RobotContainer}.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class AlgaeIntakeSubsystem extends SubsystemBase implements DashboardPublisher {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  private final Alert rollerDisconnectedAlert;
  private final Alert angleDisconnectedAlert;

  /**
   * Creates a new AlgaeIntakeSubsystem.
   * @param io
   */
  public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
    this.io = io;

    rollerDisconnectedAlert =
        new Alert(
            "Disconnected roller motor on algae intake.", AlertType.kError);
    angleDisconnectedAlert =
        new Alert(
            "Disconnected angle motor on algae intake.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if(!Constants.minimalLogging) Logger.processInputs("AlgaeIntake/Intake", inputs);
    if(!Constants.minimalLogging) Logger.recordOutput("AlgaeIntake/Ye", 0.0);

    rollerDisconnectedAlert.set(!inputs.rollerConnected);
    angleDisconnectedAlert.set(!inputs.angleConnected);
  }

  public void apply(double intakeVelocity, Rotation2d anglePosition) {
    io.setAnglePosition(anglePosition
      .plus(AlgaeIntakeConstants.ANGLE_OFFSET));
    io.setRollerVelocity(intakeVelocity);
  }

  public void apply(double intakeVelocity, Rotation2d anglePosition, boolean resist) {
    io.setAnglePosition(anglePosition
      .plus(AlgaeIntakeConstants.ANGLE_OFFSET), resist);
    io.setRollerVelocity(intakeVelocity);
  }

  public void setRollerSpeed(double intakeVelocity) {
    io.setRollerVelocity(intakeVelocity);
  }

  public void setTargetAngle(Rotation2d targetAngle) {
    io.setAnglePosition(targetAngle
      .plus(AlgaeIntakeConstants.ANGLE_OFFSET));
  }

  public void runCharacterization(double output) {
    io.setRollerOpenLoop(output);
    io.setAnglePosition(new Rotation2d());
  }

  public void stop() {
    io.setRollerOpenLoop(0.0);
    io.setAngleOpenLoop(0.0);
  }

  public Rotation2d getArmPosition() {
    return inputs.anglePosition
      .minus(AlgaeIntakeConstants.ANGLE_OFFSET)
      .div(AlgaeIntakeConstants.ANGLE_GEAR_RATIO);
  }

  public Rotation2d getAngleMotorPosition() {
    return inputs.anglePosition;
  }

  public double getRollerVelocity() {
    return inputs.rollerVelocity;
  }

  public double getRollerCurrent() {
    return inputs.rollerCurrent;
  }

  public AlgaeIntakeIOInputsAutoLogged getInputs() {
    return inputs;
  }

  private ShuffleboardTab tab;

  private GenericEntry armAngleEntry, armSetpointEntry;

  @Override
  public void initTab() {
    tab = Shuffleboard.getTab("AlgaeIntake");

    armAngleEntry = tab.add("Arm Angle", inputs.currentAngle)
      .withPosition(0, 0)
      .withSize(4, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    armSetpointEntry = tab.add("Arm Setpoint", inputs.currentAngle)
      .withPosition(0, 1)
      .withSize(4, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  }

  @Override
  public void updateTab() {
    armAngleEntry.setDouble(inputs.currentAngle);
    armSetpointEntry.setDouble(inputs.currentSetpoint);
  }
}
