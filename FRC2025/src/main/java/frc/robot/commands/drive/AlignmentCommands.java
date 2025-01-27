// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class AlignmentCommands {
    public static Command alignToReefSide(DriveSubsystem driveSubsystem, int side) {
        return new AlignToPose(
            driveSubsystem,
            () -> FieldConstants.REEF_POSITIONS[0][side],
            new Translation2d(0, 0.5));
    }
}
