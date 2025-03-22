// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LoggedTunableNumber;

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

    private static final Map<Integer, Double> tagRotMap = new HashMap<>();
    static {
        tagRotMap.put(6, Units.degreesToRadians(210));
        tagRotMap.put(7, Units.degreesToRadians(270));
        tagRotMap.put(8, Units.degreesToRadians(330));
        tagRotMap.put(9, Units.degreesToRadians(30));
        tagRotMap.put(10, Units.degreesToRadians(90));
        tagRotMap.put(11, Units.degreesToRadians(150));

        tagRotMap.put(19, Units.degreesToRadians(210));
        tagRotMap.put(18, Units.degreesToRadians(270));
        tagRotMap.put(17, Units.degreesToRadians(330));
        tagRotMap.put(22, Units.degreesToRadians(30));
        tagRotMap.put(21, Units.degreesToRadians(90));
        tagRotMap.put(20, Units.degreesToRadians(150));
    }

    /**
     * Reef Positions:
     * <p> ======BR=BL=====
     * <p>=LBL/+++++++\RBR=
     * <p>LBR/+++++++++\RBL
     * <p>LFL\+++++++++/RFR
     * <p>=LFR\+++++++/RFL=
     * <p> ======FL=FR=====
     */
    public static enum ReefPosition {
        CLOSEST, CLOSEST_LEFT, CLOSEST_RIGHT,
        FL,  FR,
        LFL, LFR,
        LBL, LBR,
        BL,  BR,
        RBL, RBR,
        RFL, RFR
    }

    public static Command pathfindAndAlignToReef(DriveSubsystem driveSubsystem, ReefPosition position) {
        return Commands.runOnce(() -> {
            Pose2d targetPose = getReefPositionPose(position, driveSubsystem.getPose());

            new PathfindToPose(driveSubsystem, () -> targetPose, 0.0)
                .andThen(alignToPose(driveSubsystem, () -> targetPose, 0.05));
        }, driveSubsystem);
    }

    public static Command alignToClosestReefPosition(DriveSubsystem driveSubsystem) {
        return Commands.runOnce(() -> {
            List<Pose2d> reefPoses = new ArrayList<>();
            for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
                reefPoses.add(pose[0]);
                reefPoses.add(pose[1]);
            }
            Pose2d targetPose = driveSubsystem.getPose().nearest(reefPoses);

            alignToPose(driveSubsystem, () -> targetPose, 0.05).schedule();
        }, driveSubsystem);
    }

    public static Command alignToReefPosition(DriveSubsystem driveSubsystem, ReefPosition position) {
        return Commands.runOnce(() -> {
            Pose2d targetPose = getReefPositionPose(position, driveSubsystem.getPose());
            alignToPose(driveSubsystem, () -> targetPose, 0.05).schedule();
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

    public static Command alignToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Translation2d wantedTranslation, int camIndex) {
        PIDController offsetPID = new PIDController(2.7, 0.0, 0.0);
        PIDController distancePID = new PIDController(6.0, 0.0, 0.0);
        PIDController rotationPID = new PIDController(3.5, 0.0, 0.0);
        rotationPID.enableContinuousInput(0, Math.PI * 2);

        offsetPID.setTolerance(0.01);
        distancePID.setTolerance(0.01);
        rotationPID.setTolerance(0.01);
        
        return Commands.startRun(() -> {

        }, () -> {
            if(tagRotMap.containsKey(visionSubsystem.getTargetID(camIndex))
                    || tagRotMap.containsKey(visionSubsystem.getTargetID(1 - camIndex))) {

                int correctCam = tagRotMap.containsKey(visionSubsystem.getTargetID(1 - camIndex))
                    && !tagRotMap.containsKey(visionSubsystem.getTargetID(camIndex)) ?
                    1 - camIndex : camIndex;

                Pose3d targetPose;
                if(correctCam != camIndex) {
                    targetPose = visionSubsystem.getTargetPose(camIndex).times(1.5);
                } else {
                    targetPose = visionSubsystem.getTargetPose(camIndex);
                }

                Logger.recordOutput("Alignment/tagPos", targetPose);

                double offsetOutput = offsetPID.calculate(targetPose.getY(), wantedTranslation.getY());
                double distanceOutput = distancePID.calculate(targetPose.getX(), wantedTranslation.getX());
                double rotationOutput = rotationPID.calculate(MathUtil.inputModulus(driveSubsystem.getRotation().getRadians(), 0, MathConstants.TAU),
                    tagRotMap.get(visionSubsystem.getTargetID(correctCam)));

                driveSubsystem.runVelocity(new ChassisSpeeds(
                    MathUtil.clamp(-offsetOutput, -2.1, 2.1),
                    MathUtil.clamp(distanceOutput, -2.1, 2.1),
                    MathUtil.clamp(rotationOutput, -2.1, 2.1)));
            } else {
                driveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        }, driveSubsystem).until(() -> {
            return offsetPID.atSetpoint() && distancePID.atSetpoint() && rotationPID.atSetpoint();
        }).andThen(() -> { offsetPID.close(); distancePID.close(); rotationPID.close(); });
    }

    public static Pose2d getReefPositionPose(ReefPosition position, Pose2d drivePose) {
        Pose2d targetPose;
        String positionString = position.toString();
        if(positionString.startsWith("CLOSEST")) {
            List<Pose2d> reefPoses = new ArrayList<>();
            for(Pose2d[] pose : FieldConstants.REEF_POSITIONS) {
                if(position == ReefPosition.CLOSEST_LEFT) {
                    reefPoses.add(pose[0]);
                } else {
                    reefPoses.add(pose[1]);
                }
            }
            targetPose = drivePose.nearest(reefPoses);
        } else {
            Pose2d[] poses = new Pose2d[2];
            if(positionString.startsWith("F")) {
                poses = FieldConstants.REEF_POSITIONS[0];
            } else if(positionString.startsWith("LF")) {
                poses = FieldConstants.REEF_POSITIONS[1];
            } else if(positionString.startsWith("LB")) {
                poses = FieldConstants.REEF_POSITIONS[2];
            } else if(positionString.startsWith("B")) {
                poses = FieldConstants.REEF_POSITIONS[3];
            } else if(positionString.startsWith("RB")) {
                poses = FieldConstants.REEF_POSITIONS[4];
            } else if(positionString.startsWith("RF")) {
                poses = FieldConstants.REEF_POSITIONS[5];
            }
            if (positionString.endsWith("L")) {
                targetPose = poses[0];
            } else {
                targetPose = poses[1];
            }
        }
        return targetPose;
    }
}
