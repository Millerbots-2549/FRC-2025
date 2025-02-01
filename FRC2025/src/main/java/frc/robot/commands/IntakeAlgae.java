// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private boolean algaeContacted;
  private boolean finalize;

  /** Creates a new IntakeAlgae. */
  public IntakeAlgae(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeContacted = false;
    finalize = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (finalize) {
      algaeIntakeSubsystem.apply(0.0, AlgaeIntakeConstants.INTAKE_ANGLE_UP);
    }
    if(algaeIntakeSubsystem.getRollerCurrent() > 5) {
      algaeContacted = true;
    }
    if (algaeContacted) {
      algaeIntakeSubsystem.apply(0.5,
        AlgaeIntakeConstants.INTAKE_ANGLE_DOWN.plus(Rotation2d.fromDegrees(45)));
      if (algaeIntakeSubsystem.getArmPosition().getRadians()
          < AlgaeIntakeConstants.INTAKE_ANGLE_UP.plus(Rotation2d.fromDegrees(30)).getRadians()) {
        finalize = true;
      }
    } else {
      algaeIntakeSubsystem.apply(0.5, AlgaeIntakeConstants.INTAKE_ANGLE_DOWN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finalize &&
      Math.abs(algaeIntakeSubsystem.getArmPosition().getRadians()
        - AlgaeIntakeConstants.INTAKE_ANGLE_UP.getRadians())
      < 0.1;
  }
}
