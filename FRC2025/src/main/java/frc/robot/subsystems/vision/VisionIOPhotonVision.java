// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                PhotonTrackedTarget best = result.getBestTarget();
                double bestDist = Double.POSITIVE_INFINITY;
                for(PhotonTrackedTarget t : result.getTargets()) {
                    double dist = t.bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero);
                    if (dist < bestDist) {
                        bestDist = dist;
                        best = t;
                    }
                }
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(best.getYaw()),
                        Rotation2d.fromDegrees(best.getPitch()));
                inputs.latestTargetArea = best.getArea();
                inputs.latestTargetSkew = best.getSkew();
                inputs.latestTarget3dPose = new Pose3d(
                    best.bestCameraToTarget.getTranslation(),
                    best.bestCameraToTarget.getRotation());
                inputs.latestTargetID = best.getFiducialId();
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
                inputs.latestTargetArea = 0.0;
                inputs.latestTargetSkew = 0.0;
                inputs.latestTargetID = -1;
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) {
                MultiTargetPNPResult multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (PhotonTrackedTarget target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                robotPose = new Pose3d(
                    ((robotPose.getX() - 9.0) * -1) + 9.0,
                    ((robotPose.getY() - 4.0) * -1) + 4.0,
                    robotPose.getZ(),
                    robotPose.getRotation()
                );

                // Add observation
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Ambiguity
                        multitagResult.fiducialIDsUsed.size(), // Tag count
                        totalTagDistance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}
