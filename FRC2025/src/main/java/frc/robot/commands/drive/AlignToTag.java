// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.lang.module.FindException;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private final double wantedXOffset;
  private final double wantedYOffset;

  /** Creates a new AlignToTag. */
  public AlignToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double wantedXOffset, double wantedYOffset) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.wantedXOffset = wantedXOffset;
    this.wantedYOffset = wantedYOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
