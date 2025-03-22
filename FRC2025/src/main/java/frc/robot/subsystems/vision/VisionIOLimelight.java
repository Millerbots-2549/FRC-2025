// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
     */
    public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotationSupplier = rotationSupplier;
        orientationPublisher =
                table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()),
                0.0, 0.0, new Pose3d(), -1);

        orientationPublisher.accept(new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        NetworkTableInstance.getDefault().flush();

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : megatag1Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 10; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,

                    parsePose(rawSample.value),

                    rawSample.value.length >= 17 ? rawSample.value[16] : 0.0,

                    (int) rawSample.value[8],

                    rawSample.value[10],

                    PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 10; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,

                    parsePose(rawSample.value),

                    0.0,

                    (int) rawSample.value[8],

                    rawSample.value[10],

                    PoseObservationType.MEGATAG_2));
        }

        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}
