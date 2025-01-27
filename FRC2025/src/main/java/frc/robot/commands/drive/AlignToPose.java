// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPose extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;
  Supplier<Pose2d> targetPose;
  Supplier<Pose2d> currentPose;

  Command pathfindingCommand;

  PIDController xController;
  PIDController yController;
  PIDController thetaController;

  /** Creates a new AlignToPose. */
  public AlignToPose(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPose, Translation2d pathfindOffset) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;

    this.currentPose = () -> driveSubsystem.getPose();

    xController = new PIDController(2.7, 0, 0);
    xController.setTolerance(0.1);
    yController = new PIDController(2.7, 0, 0);
    yController.setTolerance(0.1);
    thetaController = new PIDController(1.8, 0, 0);
    thetaController.setTolerance(0.1);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double x = targetPose.get().getX() - currentPose.get().getX();
    double y = targetPose.get().getY() - currentPose.get().getY();
    if (Math.hypot(x, y) < DriveConstants.ALIGNMENT_MIN_DISTANCE) {
      pathfindingCommand = Commands.none();
    } else {
      Pose2d newTarget = targetPose.get().transformBy(new Transform2d(
        pathfindOffset, Rotation2d.kZero));
      pathfindingCommand = new PathfindToPose(driveSubsystem, () -> newTarget, 3.0);
    }

    addCommands(
      pathfindingCommand,
      buildAlignmentCommand());
  }

  public Command buildAlignmentCommand() {
    return Commands.run(() -> {
      double xError = currentPose.get().getX() - targetPose.get().getX();
      double yError = currentPose.get().getY() - targetPose.get().getY();
      double thetaError = currentPose.get().getRotation().getRadians() - targetPose.get().getRotation().getRadians();

      double x = MathUtil.clamp(xController.calculate(xError, 0), -2.0, 2.0);
      double y = MathUtil.clamp(yController.calculate(yError, 0), -2.0, 2.0);
      double theta = thetaController.calculate(thetaError, 0);

      driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, theta), driveSubsystem.getRotation()));
    }).until(() -> {
      return Math.hypot(
        xController.getPositionError(),
        yController.getPositionError()) < DriveConstants.ALIGNMENT_MIN_TRANSLATION_ERROR
        && Math.abs(thetaController.getPositionError()) < DriveConstants.ALIGNMENT_MIN_THETA_ERROR;
    });
  }
}
