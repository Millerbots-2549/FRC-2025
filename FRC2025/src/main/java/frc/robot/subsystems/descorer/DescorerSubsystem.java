// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.descorer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DescorerSubsystem extends SubsystemBase {
  private DescorerIO io;
  private DescorerIOInputsAutoLogged inputs = new DescorerIOInputsAutoLogged();

  /** Creates a new DescorerSubsystem. */
  public DescorerSubsystem(DescorerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    SmartDashboard.putNumber("Descorer Wrist Angle", inputs.wristPosition.getRadians());
  }

  public void runRoller(double velocity) {
    io.applyRollerVelocity(velocity);
  }

  public void applyWristSetpoint(Rotation2d setpoint) {
    io.applyWristSetpoint(setpoint);
  }
}
