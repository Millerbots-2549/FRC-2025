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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.commands.auto.PathfindingCommands;
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

        tagRotMap.put(19, Units.degreesToRadians(210 + 180));
        tagRotMap.put(18, Units.degreesToRadians(270 + 180));
        tagRotMap.put(17, Units.degreesToRadians(330 + 180));
        tagRotMap.put(22, Units.degreesToRadians(30 + 180));
        tagRotMap.put(21, Units.degreesToRadians(90 + 180));
        tagRotMap.put(20, Units.degreesToRadians(150 + 180));
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

    /**
     * Pathfinds to a reef position, and then precisely aligns using the pose estimator
     * @param driveSubsystem The drive subsystem
     * @param position The reef position to align to
     * @return A command to pathfind and align to a reef position
     */
    public static Command pathfindAndAlignToReef(DriveSubsystem driveSubsystem, ReefPosition position) {
        return PathfindingCommands.pathfindToPoint(getReefPositionPose(position, driveSubsystem.getPose()))
            .andThen(alignToPose(driveSubsystem, () -> getReefPositionPose(position, driveSubsystem.getPose()), 0.05));
    }

    /**
     * Precisely aligns to the reef position that the robot is currently closest to
     * @param driveSubsystem The drive subsystem
     * @return A command to align to the closest reef position
     */
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

    /**
     * Precisely aligns to a specific reef position
     * @param driveSubsystem The drive subsystem
     * @param position The reef position
     * @return A command to align to the reef position
     */
    public static Command alignToReefPosition(DriveSubsystem driveSubsystem, ReefPosition position) {
        return Commands.runOnce(() -> {
            Pose2d targetPose = getReefPositionPose(position, driveSubsystem.getPose());
            alignToPose(driveSubsystem, () -> targetPose, 0.05).schedule();
        }, driveSubsystem);
    }

    /**
     * Aligns to a specific pose using three PID controllers and the drive pose estimator
     * <p> You should use pathfinding to get roughly to the right position, and then use this to align precisely to the right position.
     * @param driveSubsystem The drive subsystem
     * @param targetPoseSupplier The target pose
     * @param tolerance The tolerance for the pose
     * @return A command to align to the pose
     */
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

    /**
     * ALigns to an april tag using one of the cameras on the right side of the robot.
     * @param driveSubsystem The drive subsystem
     * @param visionSubsystem The vision subsystem
     * @param wantedTranslation The optimal position of the april tag relative to the camera
     * @param camIndex The index of the camera to use (0 or 1)
     * @return A command to align to a tag
     */
    public static Command alignToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Translation2d wantedTranslation, int camIndex) {
        PIDController offsetPID = new PIDController(6.3, 0.0, 0.05);
        PIDController distancePID = new PIDController(5.3, 0.0, 0.05);
        PIDController rotationPID = new PIDController(3.5, 0.0, 0.0);
        rotationPID.enableContinuousInput(0, Math.PI * 2);

        double CAM_0_ANGLE = Units.degreesToRadians(0);
        double CAM_1_ANGLE = Units.degreesToRadians(30);
        
        return Commands.startRun(() -> {
            offsetPID.setTolerance(0.005);
            distancePID.setTolerance(0.005);
            rotationPID.setTolerance(0.01);
        }, () -> {
            // Checks if either camera contains a reef tag
            int targetID = visionSubsystem.getTargetID(camIndex);
            int alternateTargetID = visionSubsystem.getTargetID(1 - camIndex);
            if(tagRotMap.containsKey(targetID)
                    || tagRotMap.containsKey(alternateTargetID)) {

                // Finds which camera contains the target
                int correctCam = tagRotMap.containsKey(alternateTargetID)
                    && !tagRotMap.containsKey(targetID) ?
                    1 - camIndex : camIndex;

                // Finds the target pose
                Pose3d targetPose;
                if(correctCam != camIndex) {
                    // If the wrong camera has the tag, then move the robot sideways until the correct camera can see it.
                    if(camIndex == 0) {
                        targetPose = new Pose3d(wantedTranslation.getX(), 0.2, 0.0, Rotation3d.kZero);
                    } else {
                        targetPose = new Pose3d(wantedTranslation.getX(), -0.2, 0.0, Rotation3d.kZero);
                    }
                } else {
                    targetPose = visionSubsystem.getTargetPose(camIndex);
                }

                // Compensate for the rotation of the cameras
                targetPose = targetPose.rotateBy(new Rotation3d(0.0, 0.0, camIndex == 0 ? CAM_0_ANGLE : CAM_1_ANGLE));

                // Log the compensated target position
                Logger.recordOutput("Alignment/tagPos", targetPose);

                // Calculates the outputs from the PID controllers.
                double offsetOutput = offsetPID.calculate(targetPose.getY(), wantedTranslation.getY());
                double distanceOutput = distancePID.calculate(targetPose.getX(), wantedTranslation.getX());
                double rotationOutput = rotationPID.calculate(MathUtil.inputModulus(driveSubsystem.getRotation().getRadians(), 0, MathConstants.TAU),
                    tagRotMap.get(visionSubsystem.getTargetID(correctCam)));

                // Drives the robot
                driveSubsystem.runVelocity(new ChassisSpeeds(
                    MathUtil.clamp(-offsetOutput, -4.3, 4.3),
                    MathUtil.clamp(distanceOutput, -4.3, 4.3),
                    MathUtil.clamp(rotationOutput, -4.0, 4.0)));
            } else {
                // Stops the robot if neither camera can see the apriltag
                driveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        }, driveSubsystem).until(() -> {
            // Stops the command when the robot is fully aligned
            return offsetPID.atSetpoint() && distancePID.atSetpoint() && rotationPID.atSetpoint();
        }).andThen(() -> { offsetPID.close(); distancePID.close(); rotationPID.close(); });
    }

    /**
     * Aligns to the left branch of the reef based on the apriltag
     * @param driveSubsystem The drive subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to align to the tag
     */
    public static Command alignToTagLeft(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        return AlignmentCommands.alignToTag(driveSubsystem, visionSubsystem, new Translation2d(0.338, -0.075), 0);
    };

    /**
     * Aligns to the right branch of the reef based on the apriltag
     * @param driveSubsystem The drive subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to align to the tag
     */
    public static Command alignToTagRight(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        return AlignmentCommands.alignToTag(driveSubsystem, visionSubsystem, new Translation2d(0.359, 0.281), 1);
    };

    public static Command alignToStation(DriveSubsystem driveSubsystem) {
        return PathfindingCommands.pathfindToPoint(new Pose2d(1.522, 6.650, Rotation2d.fromDegrees(125)))
            .andThen(alignToPose(driveSubsystem, () -> new Pose2d(1.190, 7.022, Rotation2d.fromDegrees(125)), 0.1));
    }

    /**
     * Gets the pose that the robot should be at in order to score into the given reef position
     * @param position The reef position to get
     * @param drivePose The current position of the robot
     * @return The pose that the robot should be at to score
     */
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
