// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class AlignmentCommands {
    private static Pose2d targetPose;

    public static Command alignToReef(DriveSubsystem driveSubsystem) {
        return Commands.runOnce(() -> {
            List<Pose2d> reefPoses = new ArrayList<>();
            for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
                reefPoses.add(new Pose2d(pose[0].getTranslation(), pose[0].getRotation()));
                reefPoses.add(new Pose2d(pose[1].getTranslation(), pose[1].getRotation()));
            }
            targetPose = driveSubsystem.getPose().nearest(reefPoses);

            new AlignToPose(driveSubsystem, () -> targetPose, new Translation2d(0, 0.3)).schedule();;
        }, driveSubsystem);
    }

    public static Command alignToReefTagLeft(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        return new AlignToTag(driveSubsystem, visionSubsystem, 0, 0);
    }
}
