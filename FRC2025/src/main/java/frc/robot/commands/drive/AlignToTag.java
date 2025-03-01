// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private final double wantedXOffset;
  private final double wantedTagArea;
  
  private final PIDController offsetController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController distanceController = new PIDController(5.0, 0.0, 0.0);

  private boolean finish = false;

  /** Creates a new AlignToTag. */
  public AlignToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double wantedXOffset, double wantedTagArea) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.wantedXOffset = wantedXOffset;
    this.wantedTagArea = wantedTagArea;

    this.finish = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentXOffset = visionSubsystem.getTargetX(0).getDegrees();

    double xOutput = offsetController.calculate(currentXOffset, wantedXOffset);
    xOutput = MathUtil.clamp(xOutput, -0.05, 0.05);

    ChassisSpeeds speeds = new ChassisSpeeds(xOutput, 0, 0);

    driveSubsystem.runVelocity(speeds);

    if (offsetController.atSetpoint()) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
