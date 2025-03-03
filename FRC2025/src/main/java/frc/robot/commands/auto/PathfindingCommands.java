// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class PathfindingCommands {
    public static Command pathfindToPoint(DriveSubsystem driveSubsystem, Pose2d pose) {
        return null;
    }
    public static Command pathfindToPointWhileAvoiding(DriveSubsystem driveSubsystem, Pose2d pose, Supplier<List<Pair<Translation2d, Translation2d>>> obstacleSupplier) {
        return null;
    }
}
