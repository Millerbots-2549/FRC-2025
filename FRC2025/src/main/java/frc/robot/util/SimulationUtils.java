// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SimulationUtils {
    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
        odometryTimeStamps[i] =
            Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }

    public void addStartGamePieces() {
        
    }

    public static Pose3d getAlgaeIntakePose(double intakeRotationRadians) {
        Pose3d algaeIntakePose = new Pose3d(
            new Translation3d(
                0.13 + (Math.cos(intakeRotationRadians + Units.degreesToRadians(118)) * 0.28),
                0,
                0.235 + (Math.sin(intakeRotationRadians + Units.degreesToRadians(116)) * -0.26)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),
                intakeRotationRadians,
                Units.degreesToRadians(0)));
        return algaeIntakePose;
    }

    public static Pose3d getElevatorMiddlePose(double height) {
        Pose3d algaeIntakePose = new Pose3d(
            new Translation3d(
                0,
                0,
                height / 1.22
            ),
            new Rotation3d());
        return algaeIntakePose;
    }

    public static Pose3d getElevatorInnerPose(double height) {
        Pose3d algaeIntakePose = new Pose3d(
            new Translation3d(
                0,
                0,
                height * 1.47
            ),
            new Rotation3d());
        return algaeIntakePose;
    }

    public static Pose2d[][] generateReefPoses(Translation2d reefCenter, double reefRadius, double robotRadius, Translation2d translationOffset, Rotation2d rotationOffset) {
        Pose2d[][] reefPoses = new Pose2d[6][2];
        for (int i = 0; i < 6; i++) {
            double normX = Math.cos(Math.toRadians(60 * i));
            double normY = Math.sin(Math.toRadians(60 * i));
            reefPoses[i][0] = new Pose2d(
                reefCenter.getX() + (normX * (reefRadius + robotRadius)) + (translationOffset.getX() * normX) + (0.2 * normY),
                reefCenter.getY() + (normY * (reefRadius + robotRadius)) + (translationOffset.getY() * normY) + (-0.2 * normX),
                Rotation2d.fromDegrees(rotationOffset.getDegrees() + (60 * i))
            );
            reefPoses[i][1] = new Pose2d(
                reefCenter.getX() + (normX * (reefRadius + robotRadius)) + (translationOffset.getX() * normX) + (0.5 * normY),
                reefCenter.getY() + (normY * (reefRadius + robotRadius)) + (translationOffset.getX() * normY) + (-0.5 * normX),
                Rotation2d.fromDegrees(rotationOffset.getDegrees() + (60 * i))
            );
        }
        return reefPoses;
    }
}
