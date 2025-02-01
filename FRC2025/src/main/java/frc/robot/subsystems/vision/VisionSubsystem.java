// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.ANGULAR_STD_DEV_MEGATAG2_FACTOR;
import static frc.robot.Constants.VisionConstants.APRIL_TAG_LAYOUT;
import static frc.robot.Constants.VisionConstants.CAMERA_STD_DEV_FACTORS;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_BASELINE;
import static frc.robot.Constants.VisionConstants.LINEAR_STD_DEV_MEGATAG2_FACTOR;
import static frc.robot.Constants.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_Z_ERROR;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase {
    @FunctionalInterface
    public interface VisionConsumer {
        void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + i, inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            if(io[cameraIndex].useAprilTags()) {
                // Add tag poses
                for (int tagId : inputs[cameraIndex].tagIds) {
                    var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
                    if (tagPose.isPresent()) {
                        tagPoses.add(tagPose.get());
                    }
                }

                // Loop over pose observations
                for (var observation : inputs[cameraIndex].poseObservations) {
                    // Check whether to reject pose
                    boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                            || (observation.tagCount() == 1
                                    && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                            || Math.abs(observation.pose().getZ()) > MAX_Z_ERROR // Must have realistic Z coordinate

                            // Must be within the field boundaries
                            || observation.pose().getX() < 0.0
                            || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
                            || observation.pose().getY() < 0.0
                            || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();

                    // Add pose to log
                    robotPoses.add(observation.pose());
                    if (rejectPose) {
                        robotPosesRejected.add(observation.pose());
                    } else {
                        robotPosesAccepted.add(observation.pose());
                    }

                    // Skip if rejected
                    if (rejectPose) {
                        continue;
                    }

                    // Calculate standard deviations
                    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                    double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                    double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                    if (observation.type() == PoseObservationType.MEGATAG_2) {
                        linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                        angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                    }
                    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
                        linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
                        angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
                    }

                    // Send vision observation
                    
                    consumer.addVisionMeasurement(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                            
                }
            } else { // Subsystem doesn't use april tags
                for (var observation : inputs[cameraIndex].poseObservations) {
                    robotPoses.add(observation.pose());

                    double linearStdDev = LINEAR_STD_DEV_BASELINE;
                    double angularStdDev = ANGULAR_STD_DEV_BASELINE;

                    consumer.addVisionMeasurement(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                }
            }

            if(io[cameraIndex].useAprilTags()) {
                // Log camera datadata
                Logger.recordOutput(
                        "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
            }
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        if (!allTagPoses.isEmpty()) {
            Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        }
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
}
