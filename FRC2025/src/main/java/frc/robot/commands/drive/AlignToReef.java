// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToReef extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;
  Pose2d targetPose;
  Supplier<Pose2d> currentPose;

  Command pathfindingCommand;

  PIDController xController;
  PIDController yController;
  PIDController thetaController;

  int side;

  /** Creates a new AlignToPose. */
  public AlignToReef(DriveSubsystem driveSubsystem, int side, Translation2d pathfindOffset) {
    this.driveSubsystem = driveSubsystem;

    this.currentPose = () -> driveSubsystem.getPose();

    this.targetPose = new Pose2d();

    this.side = side;

    xController = new PIDController(4.8, 0, 0.8);
    xController.setTolerance(0.02);
    yController = new PIDController(4.8, 0, 0.8);
    yController.setTolerance(0.02);
    thetaController = new PIDController(3.9, 0, 0);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    initializeWithOffset(pathfindOffset);

    addCommands(
      Commands.runOnce(() -> {
        List<Pose2d> reefPoses = new ArrayList<>();
        for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
          reefPoses.add(pose[side]);
        }
        this.targetPose = currentPose.get().nearest(reefPoses);

        initializeWithOffset(pathfindOffset);
      }),
      pathfindingCommand,
      buildAlignmentCommand());
  }

  public Command buildAlignmentCommand() {
    return Commands.run(() -> {
      double xError = currentPose.get().getX() - targetPose.getX();
      double yError = currentPose.get().getY() - targetPose.getY();
      double thetaError = currentPose.get().getRotation().getRadians() - targetPose.getRotation().getRadians();

      double x = MathUtil.clamp(xController.calculate(xError, 0), -2.0, 2.0);
      double y = MathUtil.clamp(yController.calculate(yError, 0), -2.0, 2.0);
      double theta = thetaController.calculate(thetaError, 0);

      driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, theta), driveSubsystem.getRotation()));
    }).until(() -> {
      return Math.hypot(
        currentPose.get().getX() - targetPose.getX(),
        currentPose.get().getY() - targetPose.getY()) < DriveConstants.ALIGNMENT_MIN_TRANSLATION_ERROR
        && Math.abs(currentPose.get().getRotation().getRadians() - targetPose.getRotation().getRadians()) < DriveConstants.ALIGNMENT_MIN_THETA_ERROR;
    });
  }

  private void initializeWithOffset(Translation2d pathfindOffset) {
    double x = targetPose.getX() - currentPose.get().getX();
    double y = targetPose.getY() - currentPose.get().getY();
    if (Math.hypot(x, y) < DriveConstants.ALIGNMENT_MIN_DISTANCE) {
      pathfindingCommand = Commands.none();
    } else {
      Pose2d newTarget = targetPose.transformBy(new Transform2d(
        pathfindOffset, Rotation2d.kZero));
      pathfindingCommand = new PathfindToPose(driveSubsystem, () -> newTarget, 3.0);
    }
  }
}
