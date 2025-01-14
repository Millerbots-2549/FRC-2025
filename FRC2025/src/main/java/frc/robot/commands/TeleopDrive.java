// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.MathConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  DriveSubsystem driveSubsystem;

  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier rotationSupplier;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem driveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this. rotationSupplier = rotationSupplier;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d linearVelocity = getVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    double rot = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), OperatorConstants.DEADBAND);

    rot = Math.copySign(rot * rot, rot);

    ChassisSpeeds speeds = new ChassisSpeeds(
      linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
      linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
      rot * driveSubsystem.getMaxAngularSpeedRadPerSec());
    boolean isFlipped = DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Red;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      isFlipped ? driveSubsystem.getRotation().plus(new Rotation2d(PI))
      : driveSubsystem.getRotation());
    driveSubsystem.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getVelocityFromJoysticks(double x, double y) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), OperatorConstants.DEADBAND);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    magnitude = magnitude * magnitude;

    return new Pose2d(new Translation2d(), direction)
      .transformBy(new Transform2d(magnitude, 0.0, new Rotation2d()))
      .getTranslation();
  }
}
