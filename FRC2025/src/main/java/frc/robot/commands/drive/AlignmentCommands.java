// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public final class AlignmentCommands {
    private static LoggedTunableNumber xPIDkP = new LoggedTunableNumber("/Tuning/xPIDkP", 0.0);
    private static LoggedTunableNumber xPIDkI = new LoggedTunableNumber("/Tuning/xPIDkI", 0.0);
    private static LoggedTunableNumber xPIDkD = new LoggedTunableNumber("/Tuning/xPIDkD", 0.0);
    private static LoggedTunableNumber yPIDkP = new LoggedTunableNumber("/Tuning/yPIDkP", 0.0);
    private static LoggedTunableNumber yPIDkI = new LoggedTunableNumber("/Tuning/yPIDkI", 0.0);
    private static LoggedTunableNumber yPIDkD = new LoggedTunableNumber("/Tuning/yPIDkD", 0.0);
    private static LoggedTunableNumber thetaPIDkP = new LoggedTunableNumber("/Tuning/thetaPIDkP", 0.0);
    private static LoggedTunableNumber thetaPIDkI = new LoggedTunableNumber("/Tuning/thetaPIDkI", 0.0);
    private static LoggedTunableNumber thetaPIDkD = new LoggedTunableNumber("/Tuning/thetaPIDkD", 0.0);

    public static Command pathfindAndAlignToReef(DriveSubsystem driveSubsystem) {
        return Commands.runOnce(() -> {
            List<Pose2d> reefPoses = new ArrayList<>();
            for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
                reefPoses.add(new Pose2d(pose[0].getTranslation(), pose[0].getRotation()));
                reefPoses.add(new Pose2d(pose[1].getTranslation(), pose[1].getRotation()));
            }
            Pose2d targetPose = driveSubsystem.getPose().nearest(reefPoses);

            new PathfindAndAlignToPose(driveSubsystem, () -> targetPose, new Translation2d(0, 0.3)).schedule();
        }, driveSubsystem);
    }

    public static Command alignToClosestReefPosition(DriveSubsystem driveSubsystem) {
        return Commands.runOnce(() -> {
            List<Pose2d> reefPoses = new ArrayList<>();
            for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
                reefPoses.add(new Pose2d(pose[0].getTranslation(), pose[0].getRotation()));
                reefPoses.add(new Pose2d(pose[1].getTranslation(), pose[1].getRotation()));
            }
            Pose2d targetPose = driveSubsystem.getPose().nearest(reefPoses);

            alignToPose(driveSubsystem, () -> targetPose, 0.1).schedule();
        }, driveSubsystem);
    }

    public static Command alignToPose(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPoseSupplier, double tolerance) {
        PIDController xPID = new PIDController(0, 0, 0);
        PIDController yPID = new PIDController(0, 0, 0);
        PIDController thetaPID = new PIDController(0, 0, 0);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            xPID.setPID(xPIDkP.get(), xPIDkI.get(), xPIDkD.get());
            yPID.setPID(yPIDkP.get(), yPIDkI.get(), yPIDkD.get());
            thetaPID.setPID(thetaPIDkP.get(), thetaPIDkI.get(), thetaPIDkD.get());

            xPID.reset();
            yPID.reset();
            thetaPID.reset();
        }, () -> {
            Pose2d currentPose = driveSubsystem.getPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xError = currentPose.getX() - targetPose.getX();
            double yError = currentPose.getY() - targetPose.getY();
            double thetaError = MathUtil.angleModulus(currentPose.getRotation().getRadians())
                - MathUtil.angleModulus(targetPose.getRotation().getRadians());

            double x = xPID.calculate(xError, 0);
            double y = yPID.calculate(yError, 0);
            double theta = thetaPID.calculate(thetaError, 0);

            if(Math.hypot(x, y) > MAX_SPEED_METERS_PER_SECOND) {
                double scale = MAX_SPEED_METERS_PER_SECOND / Math.hypot(x, y);
                x *= scale;
                y *= scale;
            }

            theta = MathUtil.clamp(theta, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

            driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(x, y, theta), driveSubsystem.getRotation()));
        }, driveSubsystem).until(() -> {
            double dist = driveSubsystem.getPose().getTranslation()
                .getDistance(targetPoseSupplier.get().getTranslation());
            return dist < tolerance;
        }).andThen(() -> {xPID.close(); yPID.close(); thetaPID.close();});
    }

    public static Command alignToReefTagLeft(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        return new AlignToTag(driveSubsystem, visionSubsystem, 0, 0);
    }
}
