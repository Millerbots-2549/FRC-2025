// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private boolean intakeDown;
  private boolean finalize;

  private double downPoint;

  /** Creates a new IntakeAlgae. */
  public IntakeAlgae(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This converts the rotation into radians in the correct range
    downPoint =
        MathUtil.inputModulus(
            -AlgaeIntakeConstants.INTAKE_ANGLE_DOWN.getRadians(),
            AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
            AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
        );
    intakeDown = false;
    finalize = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the measurement to the correct range
    double absolutePosition = 
        MathUtil.inputModulus(
            algaeIntakeSubsystem.getAngleMotorPosition().getRadians(),
            AlgaeIntakeConstants.ANGLE_PID_MIN_INPUT,
            AlgaeIntakeConstants.ANGLE_PID_MAX_INPUT
        );

    SmartDashboard.putBoolean("IntakeDown", intakeDown);
    SmartDashboard.putNumber("Dist", Math.abs(absolutePosition - downPoint));

    if (finalize) {
      algaeIntakeSubsystem.apply(0.0, AlgaeIntakeConstants.INTAKE_ANGLE_UP);
      return;
    }
    if (Math.abs(absolutePosition - downPoint) < 1.4) {
      intakeDown = true;
    }
    if (intakeDown) {
      algaeIntakeSubsystem.apply(1.0,
        AlgaeIntakeConstants.INTAKE_ANGLE_DOWN.plus(Rotation2d.fromDegrees(15)));
      if (Math.abs(absolutePosition - downPoint) > 1.8) {
        finalize = true;
      }
    } else {
      algaeIntakeSubsystem.apply(1.0, AlgaeIntakeConstants.INTAKE_ANGLE_DOWN);
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
