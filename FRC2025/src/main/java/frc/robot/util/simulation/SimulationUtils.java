// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.simulation;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SimulationUtils {
    public static SimulationUtils instance = null;

    public static SimulationUtils getInstance() {
        if (instance == null) {
            instance = new SimulationUtils();
        }
        return instance;
    }

    public static final LinkedHashMap<String, Collider> gamePieceColliders = new LinkedHashMap<String, Collider>();

    static
    {
        gamePieceColliders.put("Algae", new Collider());
        gamePieceColliders.put("Coral", new Collider());
    }

    public Pose3d robotPose = new Pose3d();
    public Rotation2d robotRotation = new Rotation2d();
    public void updateRobotPose(Pose2d pose) {
        robotPose = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 0), robotPose.getRotation());
        robotRotation = pose.getRotation();
    };

    public List<Pose3d> algaePoses = new ArrayList<>();
    public void updateAlgae(List<Pose3d> poses) {
        algaePoses = poses;
    };
    public List<Pose3d> coralPoses = new ArrayList<>();
    public void updateCoral(List<Pose3d> poses) {
        coralPoses = poses;
    };

    public int intakeAlgae = 0;
    public int intakeCoral = 0;
    public void updateIntakes(IntakeSimulation algae, IntakeSimulation coral) {
        intakeAlgae = algae.getGamePiecesAmount();
        intakeCoral = coral.getGamePiecesAmount();
    };

    public int previousIntakeAlgae = 0;
    public double lastIntakeAlgaeTimestamp = 0;
    public double lastIntakeAlgaeTimePassed = 0;

    public void update() {
        if (previousIntakeAlgae < intakeAlgae) {
            lastIntakeAlgaeTimestamp = Timer.getTimestamp();
        }
        lastIntakeAlgaeTimePassed = Timer.getTimestamp() - lastIntakeAlgaeTimestamp;
        previousIntakeAlgae = intakeAlgae;
    }

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
                height < 0.65 ? 0.0 : height - 0.65
            ),
            new Rotation3d());
        return algaeIntakePose;
    }

    public static Pose3d getElevatorInnerPose(double height) {
        Pose3d algaeIntakePose = new Pose3d(
            new Translation3d(
                0,
                0,
                height
            ),
            new Rotation3d());
        return algaeIntakePose;
    }

    public List<Pose3d> generateAlgaePoses() {
        List<Pose3d> algaePosesList = SimulatedArena.getInstance().getGamePiecesByType("Algae");
        if(intakeAlgae == 1) {
            algaePosesList.add(new Pose3d(
                new Translation3d(
                0.2 + (lastIntakeAlgaeTimePassed > 0.2 ? 0.0 : MathUtil.interpolate(0.4, 0.0, lastIntakeAlgaeTimePassed / 0.2)),
                0.0,
                0.3 - (lastIntakeAlgaeTimePassed > 0.2 ? 0.0 : MathUtil.interpolate(0.15, 0.0, lastIntakeAlgaeTimePassed / 0.2))
                ).rotateBy(new Rotation3d(0, 0, robotRotation.getRadians()))
                .plus(new Translation3d(
                robotPose.getX(),
                robotPose.getY(),
                0
                )),
                new Rotation3d(0, 0, 0)
            ));
        }
        return algaePosesList;
    }
}
