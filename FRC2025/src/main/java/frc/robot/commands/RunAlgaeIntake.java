// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeIntake extends Command {
  AlgaeIntakeSubsystem algaeIntakeSubsystem;

  /** Creates a new RunAlgaeIntake. */
  public RunAlgaeIntake(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;

    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntakeSubsystem.apply(
      AlgaeIntakeConstants.ROLLER_MAX_SPEED,
      AlgaeIntakeConstants.INTAKE_ANGLE_DOWN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(
      algaeIntakeSubsystem.getArmPosition().getDegrees() - 
      AlgaeIntakeConstants.INTAKE_ANGLE_DOWN.getDegrees())
      < AlgaeIntakeConstants.INTAKE_ANGLE_TOLERANCE;
  }
}
