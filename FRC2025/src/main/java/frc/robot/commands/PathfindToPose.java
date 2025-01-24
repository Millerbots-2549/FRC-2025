// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToPose extends Command {
  Command pathCommand;

  PathConstraints constraints = new PathConstraints(
        DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        DriveConstants.MAX_ACCELERATION,
        DriveConstants.MAX_ANGULAR_VELOCITY,
        DriveConstants.MAX_ANGULAR_ACCELERATION);

  /** Creates a new PathfindToPose. */
  public PathfindToPose(DriveSubsystem driveSubsystem, Supplier<Pose2d> target, double speed) {
    pathCommand = AutoBuilder.pathfindToPose(
      target.get(), constraints);
    
      addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
    //List<Pose2d> trajectory = Pathfinding.getCurrentPath(constraints, new GoalEndState(0, Rotation2d.kZero)).getPathPoses();
    //Logger.recordOutput("Path/Trajectory", trajectory.toArray(new Pose2d[trajectory.size()]));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
