// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToPose extends SequentialCommandGroup {
  DriveSubsystem driveSubsystem;

  Command pathCommand;
  Supplier<Pose2d> targetSupplier;
  Supplier<List<PathPoint>> pathSupplier;

  PathConstraints constraints = new PathConstraints(
        DriveConstants.MAX_SPEED_METERS_PER_SECOND / 1.5,
        DriveConstants.MAX_ACCELERATION / 3,
        DriveConstants.MAX_ANGULAR_VELOCITY,
        DriveConstants.MAX_ANGULAR_ACCELERATION);

  /** Creates a new PathfindToPose. */
  public PathfindToPose(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetSupplier, double speed, Supplier<List<PathPoint>> pathSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.targetSupplier = targetSupplier;
    this.pathSupplier = pathSupplier;

    Command initPathCommand = AutoBuilder.pathfindToPose(targetSupplier.get(), constraints);
    pathCommand = AutoBuilder.pathfindToPose(targetSupplier.get(), constraints);
    Pathfinding.ensureInitialized();
    pathCommand.initialize();
    
    addCommands(
      initPathCommand.withTimeout(0.05),
      buildInitialDriveCommand().withTimeout(3),
      pathCommand);
  }

  public Command buildInitialDriveCommand() {
    return new RunCommand(() -> {
      PathPlannerPath currentPath = Pathfinding.getCurrentPath(constraints, new GoalEndState(0, targetSupplier.get().getRotation()));

      if (currentPath != null) {
        Rotation2d rotRtoP1 = new Rotation2d(Math.atan2(
          driveSubsystem.getPose().getY() - currentPath.getPoint(1).position.getY(),
          driveSubsystem.getPose().getX() - currentPath.getPoint(1).position.getX()));

        ChassisSpeeds speeds = new ChassisSpeeds(-Math.cos(rotRtoP1.getRadians()) * DriveConstants.MAX_ACCELERATION, -Math.sin(rotRtoP1.getRadians()) * DriveConstants.MAX_ACCELERATION, 0);
        if(findRotationError(currentPath) < 45) {
          driveSubsystem.runVelocity(speeds);
        }
      }
    }, driveSubsystem).until(() -> {
      PathPlannerPath currentPath = Pathfinding.getCurrentPath(constraints, new GoalEndState(0, targetSupplier.get().getRotation()));
      if (currentPath != null) {
        if (findRotationError(currentPath) > 45) {
          return true;
        }
        return false;
      }
      return true;
    });
  }

  public double findRotationError(PathPlannerPath currentPath) {
    Rotation2d rotP1toP2 = new Rotation2d(Math.atan2(
      currentPath.getPoint(1).position.getY() - currentPath.getPoint(3).position.getY(),
      currentPath.getPoint(1).position.getX() - currentPath.getPoint(3).position.getX()));

    Rotation2d rotRtoP1 = new Rotation2d(Math.atan2(
      driveSubsystem.getPose().getY() - currentPath.getPoint(1).position.getY(),
      driveSubsystem.getPose().getX() - currentPath.getPoint(1).position.getX()));
    
    return Math.abs(rotP1toP2.getDegrees() - rotRtoP1.getDegrees());
  }
}
