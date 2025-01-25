// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.algae.AlgaeIntakeIO.AlgaeIntakeGains;
import frc.robot.util.LoggedTunableNumber;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  private final Alert rollerDisconnectedAlert;
  private final Alert angleDisconnectedAlert;

  private final LoggedTunableNumber rollerKP;
  private final LoggedTunableNumber rollerKI;
  private final LoggedTunableNumber rollerKD;
  private final LoggedTunableNumber rollerKS;
  private final LoggedTunableNumber rollerKV;
  private final LoggedTunableNumber rollerKA;

  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
    this.io = io;

    rollerDisconnectedAlert =
        new Alert(
            "Disconnected roller motor on algae intake.", AlertType.kError);
    angleDisconnectedAlert =
        new Alert(
            "Disconnected angle motor on algae intake.", AlertType.kError);

    AlgaeIntakeGains gains = io.getGains();

    rollerKP = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kP", gains.rollerKP());
    rollerKI = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kI", gains.rollerKI());
    rollerKD = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kD", gains.rollerKD());
    rollerKS = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kS", gains.rollerKS());
    rollerKV = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kV", gains.rollerKV());
    rollerKA = new LoggedTunableNumber("AlgaeIntake/Roller/Gains/kA", gains.rollerKA());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake/Intake", inputs);
    Logger.recordOutput("AlgaeIntake/Ye", 0.0);

    rollerDisconnectedAlert.set(!inputs.rollerConnected);
    angleDisconnectedAlert.set(!inputs.angleConnected);

    LoggedTunableNumber.ifChanged(
      hashCode(),
      (values) -> {
        io.setGains(
          new AlgaeIntakeGains(values[0], values[1], values[2], values[3], values[4], values[5]));
      },
      rollerKP, rollerKI, rollerKD, rollerKS, rollerKV, rollerKA);
  }

  public void apply(double intakeVelocity, Rotation2d anglePosition) {
    io.setRollerVelocity(intakeVelocity);
    io.setAnglePosition(anglePosition
      .times(AlgaeIntakeConstants.ANGLE_GEAR_RATIO)
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
}
