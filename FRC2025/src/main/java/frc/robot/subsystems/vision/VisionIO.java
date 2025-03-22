// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(
            new Rotation2d(),
            new Rotation2d(),
            0.0,
            0.0,
            new Pose3d(),
            -1);
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(
        Rotation2d tx,
        Rotation2d ty,
        double area,
        double skew,
        Pose3d pose,
        int ID) {}

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {}

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION,
        QUESTNAV
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default boolean useAprilTags() { return true; }
}
