// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.descorer.DescorerSubsystem;

import static frc.robot.Constants.DescorerConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DescoreLow extends Command {
  private final DescorerSubsystem descorerSubsystem;

  private static enum CommandState {
    LOWERING,
    PUSHING
  }
  private CommandState currentState;

  /** Creates a new DescoreLow. */
  public DescoreLow(DescorerSubsystem descorerSubsystem) {
    this.descorerSubsystem = descorerSubsystem;

    addRequirements(descorerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = CommandState.LOWERING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    descorerSubsystem.applyWristSetpoint(DESCORER_ON_POSITION);
    descorerSubsystem.runRoller(-1.0);
    if (currentState == CommandState.LOWERING) {
      // If the wrist is lowering and it is close enough to the setpoint, switch state
      if (Math.abs(descorerSubsystem.getCurrentWristPosition().getRadians()
          - DESCORER_ON_POSITION.getRadians()) < 0.1) {
        currentState = CommandState.PUSHING;
      }
    } else {
      SmartDashboard.putBoolean("Skib", false);
      // If the wrist gets far enough from the setpoint, apply force downwards
      if (descorerSubsystem.getCurrentWristPosition().getRadians()
          - DESCORER_ON_POSITION.getRadians() > 0.005) {
        descorerSubsystem.applyWristSetpoint(DESCORER_ON_POSITION.minus(Rotation2d.fromRadians(0.6)));
        SmartDashboard.putBoolean("Skib", true);
      }
    }
    SmartDashboard.putNumber("YSUSHIJ", descorerSubsystem.getCurrentWristPosition().getRadians()
      - DESCORER_ON_POSITION.getRadians());
    SmartDashboard.putBoolean("Descorer Pushing", currentState == CommandState.PUSHING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
