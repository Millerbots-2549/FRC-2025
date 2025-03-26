// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.util.Triad;
import frc.robot.util.vision.QuestNav;

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

    private final QuestNav questNav;

    private int questReorientingCount = 0;

    /**
     * A list of poses used to initialize the position of the quest nav
     * <table>
     *  <tr><th>First</th><th>Second</th><th>Third</th></tr>
     *  <tr>
     *   <th>The measured pose of the questnav</th>
     *   <th>The pose calculated from the target observation</th>
     *   <th>The confidence of the estimate (lower = better)</th>
     *  </tr>
     * </table>
    */
    private List<Triad<Pose2d, Pose2d, Double>> initializationPoses = new LinkedList<>();

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

        questNav = RobotContainer.questNav;
    }

    /**
     * Returns the best target of the camera selected
     * 
     * @param cameraIndex The index of the camera to use.
     * @return The {@link TargetObservation target}
     */
    public TargetObservation getTarget(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation;
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Returns the Y angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetY(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.ty();
    }

    /**
     * Returns the area of the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public double getTargetArea(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.area();
    }

    /**
     * Returns the skew of the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public double getTargetSkew(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.skew();
    }

    /**
     * Returns the 3d pose of the best target, which can be used for simple servoing with vision.
     * <p><strong>The output may be null if a 3d pose was not detected.</strong>
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Pose3d getTargetPose(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.pose();
    }

    /**
     * Returns the ID of the best target
     *
     * @param cameraIndex The index of the camera to use.
     */
    public int getTargetID(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.ID();
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

                Pose2d questPose = questNav.getPose();

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
                            
                    // Only add a pose to the initialization poses if the observation was calculated with april tags
                    initializationPoses.add(Triad.of(
                        questPose, // The pose of the questnav at this time
                        observation.pose().toPose2d(), // The pose of the current observation
                        (linearStdDev + angularStdDev) / 2)); // The confidence
                }
            } else { // Subsystem doesn't use april tags (Only questnav)
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
        
        if(questReorientingCount == 0) initializeQuestnav();
    }

    @SuppressWarnings("unused")
    public boolean initializeQuestnav() {
        if (initializationPoses.size() < INITIALIZATION_MINIMUM_OBSERVATIONS
                && Constants.enableQuestPoseInitialization) {
            Pose2d bestPose = new Pose2d();
            double bestConfidence = Double.POSITIVE_INFINITY;

            double totalConfidence = 0.0;

            double avgPoseDeltaX = 0.0;
            double avgPoseDeltaY = 0.0;
            double avgPoseDeltaRot = 0.0;

            double totalTranslationError = 0.0;
            double totalRotationError = 0.0;

            for(Triad<Pose2d, Pose2d, Double> initPose : initializationPoses) {
                Pose2d poseDelta = new Pose2d(
                    initPose.getSecond().getX() - initPose.getFirst().getX(),
                    initPose.getSecond().getY() - initPose.getFirst().getY(),
                    initPose.getSecond().getRotation().minus(initPose.getFirst().getRotation())
                );

                totalConfidence += (1 - initPose.getThird());
                avgPoseDeltaX += bestPose.getX() * (1 - initPose.getThird());
                avgPoseDeltaY += bestPose.getY() * (1 - initPose.getThird());
                avgPoseDeltaRot += bestPose.getRotation().getRadians() * (1 - initPose.getThird());

                totalTranslationError += initPose.getFirst().getTranslation()
                    .getDistance(initPose.getSecond().getTranslation());
                totalRotationError += Math.abs(initPose.getFirst().getRotation().getRadians()
                    - initPose.getSecond().getRotation().getRadians());
            }

            avgPoseDeltaX /= totalConfidence;
            avgPoseDeltaY /= totalConfidence;
            avgPoseDeltaRot /= totalConfidence;

            Pose2d currentPose = questNav.getPose();

            // Set new questnav position
            questNav.setPosition(new Pose2d(
                currentPose.getX() + avgPoseDeltaX,
                currentPose.getY() + avgPoseDeltaY,
                Rotation2d.fromRadians(currentPose.getRotation().getRadians() + avgPoseDeltaRot)));

            totalTranslationError /= initializationPoses.size();
            totalRotationError /= initializationPoses.size();

            // Log error values
            Logger.recordOutput("Vision/Initialization/TranslationError", totalTranslationError);
            Logger.recordOutput("Vision/Initialization/RotationError", totalRotationError);

            initializationPoses.clear();
            
            questReorientingCount++;

            return true;
        } else {
            return false;
        }
    }
}
