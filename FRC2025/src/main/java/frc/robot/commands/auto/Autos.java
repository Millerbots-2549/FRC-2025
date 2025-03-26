// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DescorerConstants;
import frc.robot.commands.drive.AlignmentCommands;
import frc.robot.commands.manipulator.DescoreHigh;
import frc.robot.commands.manipulator.DescoreLow;
import frc.robot.subsystems.descorer.DescorerSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorLevel;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
@SuppressWarnings("unused")
public class Autos {
    public static DriveSubsystem driveSubsystem;
    public static ElevatorSubsystem elevatorSubsystem;
    public static DescorerSubsystem descorerSubsystem;
    public static VisionSubsystem visionSubsystem;

    public static final Map<String, Pose2d> DESCORE_POSES = new HashMap<>();
    static {
        DESCORE_POSES.put("Back Left", new Pose2d(5.09, 5.09, Rotation2d.fromDegrees(150)));
        DESCORE_POSES.put("Front Left", new Pose2d(4.172, 5.835, Rotation2d.fromDegrees(-110.73)));
    }

    public static final void setSubsystems(DriveSubsystem driveSubsystemA, ElevatorSubsystem elevatorSubsystemA, DescorerSubsystem descorerSubsystemA, VisionSubsystem visionSubsystemA) {
        driveSubsystem = driveSubsystemA;
        elevatorSubsystem = elevatorSubsystemA;
        descorerSubsystem = descorerSubsystemA;
        visionSubsystem = visionSubsystemA;
    }

    /**
     * Aligns to the left branch of a side of the reef
     * @param level The level to raise the elevator to
     * @param seconds The amount of seconds to align for
     * @param driveSubsystem The drive subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to align to the left branch of a side of the reef
     */
    private static final Command alignReefLeft(ElevatorLevel level, double seconds, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        return AlignmentCommands.alignToTagLeft(driveSubsystem, visionSubsystem)
            .alongWith(Commands.run(() -> elevatorSubsystem.moveToLevel(level)))
            .withTimeout(seconds);
    }

    /**
     * Aligns to the right branch of a side of the reef
     * @param level The level to raise the elevator to
     * @param seconds The amount of seconds to align for
     * @param driveSubsystem The drive subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to align to the right branch of a side of the reef
     */
    private static final Command alignReefRight(ElevatorLevel level, double seconds, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        return AlignmentCommands.alignToTagRight(driveSubsystem, visionSubsystem)
            .alongWith(Commands.run(() -> elevatorSubsystem.moveToLevel(level)))
            .withTimeout(seconds);
    }

    /** Descores a lower algae */
    private static final Command descoreLow(DescorerSubsystem descorerSubsystem) {
        return new DescoreLow(descorerSubsystem).withTimeout(1.5);
    }

    /** Descores an upper algae */
    private static final Command descoreHigh(DescorerSubsystem descorerSubsystem, ElevatorSubsystem elevatorSubsystem) {
        return new DescoreHigh(descorerSubsystem, elevatorSubsystem).withTimeout(1.2);
    }

    /** Outtakes a coral */
    private static final Command outtake(ElevatorSubsystem elevatorSubsystem) {
        return Commands.run(() -> elevatorSubsystem.runIntake(0.6), elevatorSubsystem).withTimeout(0.6);
    }

    /** Starts the elevator intake */
    private static final Command startIntake(ElevatorSubsystem elevatorSubsystem) {
        return Commands.runOnce(() -> elevatorSubsystem.runIntake(0.3), elevatorSubsystem);
    }

    /** Stops the elevator intake */
    private static final Command stopIntake(ElevatorSubsystem elevatorSubsystem) {
        return Commands.runOnce(() -> elevatorSubsystem.runIntake(0.0), elevatorSubsystem);
    }

    /** Runs the elevator intake very slowly */
    private static final Command slowIntake(ElevatorSubsystem elevatorSubsystem) {
        return Commands.runOnce(() -> elevatorSubsystem.runIntake(0.05), elevatorSubsystem);
    }

    /** Lowers the elevator all the way */
    private static final Command lowerElevator(ElevatorSubsystem elevatorSubsystem) {
        return Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR), elevatorSubsystem);
    }

    /**
     * Follows a path that picks up a coral from the coral station
     * @param path The path to follow
     * @param elevatorUpTime The timestamp for the elevator to raise
     * @param elevatorDownTime The timestamp for the elevator to lower
     * @param elevatorSubsystem The elevator subsystem
     * @return A command to follow a path that passes the coral station
     */
    private static final Command followStationPath(PathPlannerPath path, double elevatorUpTime, double elevatorDownTime, ElevatorSubsystem elevatorSubsystem) {
        return new ParallelCommandGroup(
            AutoBuilder.followPath(path),
            Commands.run(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR), elevatorSubsystem).withTimeout(elevatorUpTime)
                .andThen(Commands.run(() -> elevatorSubsystem.moveToStation())).withTimeout(elevatorDownTime - elevatorUpTime)
                .andThen(lowerElevator(elevatorSubsystem)));
    }

    /**
     * Runs a full cycle
     * @param stationPath The path to the coral station
     * @param level The level to score
     * @param left If the robot should align to the left branch
     * @param driveSubsystem The drive subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to run a full cycle
     */
    private static final Command runCycle(Command stationPath, ElevatorLevel level, boolean left, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        return new SequentialCommandGroup(
            stationPath,
            left ? alignReefLeft(level, 1.0, driveSubsystem, elevatorSubsystem, visionSubsystem)
            : alignReefRight(level, 1.2, driveSubsystem, elevatorSubsystem, visionSubsystem),
            outtake(elevatorSubsystem),
            stopIntake(elevatorSubsystem),
            lowerElevator(elevatorSubsystem)
        );
    }

    // ######################################################################## //
    // ------------------------------[AUTOS]----------------------------------- //
    // ######################################################################## //

    public static final Command oneCoralOneDescore(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem) {
        PathPlannerPath part2, part3;
        try {
            part2 = PathPlannerPath.fromPathFile("1D2C Part 2");
            part3 = PathPlannerPath.fromPathFile("1D2C Part 3");
            return new SequentialCommandGroup(
                Commands.runOnce(() -> { descorerSubsystem.applyWristSetpoint(DescorerConstants.DESCORER_ON_POSITION); descorerSubsystem.runRoller(1.0); }),
                PathfindingCommands.pathfindToPoint(DESCORE_POSES.get("Back Left")),
                Commands.runOnce(() -> descorerSubsystem.applyWristSetpoint(DescorerConstants.DESCORER_OFF_POSITION), descorerSubsystem),
                AutoBuilder.followPath(part2),
                alignReefLeft(ElevatorLevel.L3, 2.5, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                stopIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                followStationPath(part3, 1.5, 5.5, elevatorSubsystem),
                alignReefRight(ElevatorLevel.L3, 2.0, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command oneDescoreTwoCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem) {
        PathPlannerPath part1, part2, part3;
        try {
            part1 = PathPlannerPath.fromPathFile("1D3C Part 1");
            part2 = PathPlannerPath.fromPathFile("1D3C Part 2");
            part3 = PathPlannerPath.fromPathFile("1D3C Part 3");
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                descoreLow(descorerSubsystem),
                AutoBuilder.followPath(part2),
                alignReefLeft(ElevatorLevel.L3, 3, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                stopIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part3, 2.0, 4.3, elevatorSubsystem),
                    ElevatorLevel.L3, false, driveSubsystem, elevatorSubsystem, visionSubsystem),
                runCycle(followStationPath(part3, 2.0, 4.3, elevatorSubsystem),
                    ElevatorLevel.L2, true, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command threeCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem) {
        PathPlannerPath part2, part3;
        try {
            part2 = PathPlannerPath.fromPathFile("0D3C Part 2");
            part3 = PathPlannerPath.fromPathFile("0D3C Part 3");
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindToPoint(new Pose2d(5.0, 5.45, Rotation2d.fromDegrees(330))),
                alignReefRight(ElevatorLevel.L2, 1.0, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem).withDeadline(new WaitCommand(0.4)),
                slowIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part2, 0.9, 4.0, elevatorSubsystem),
                    ElevatorLevel.L2, true, driveSubsystem, elevatorSubsystem, visionSubsystem),
                slowIntake(elevatorSubsystem),
                runCycle(followStationPath(part3, 0.8, 3.6, elevatorSubsystem),
                    ElevatorLevel.L2, false, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command twoCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem) {
        PathPlannerPath part1, part2;
        try {
            part1 = PathPlannerPath.fromPathFile("0D2C Part 1");
            part2 = PathPlannerPath.fromPathFile("0D2C Part 2");
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollowSlow(part1),
                alignReefRight(ElevatorLevel.L2, 2.2, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                slowIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part2, 0.9, 4.0, elevatorSubsystem),
                    ElevatorLevel.L2, true, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command branchingAuto(List<AutoAction> actions) {
        Command fullAuto = Commands.none();
        for(AutoAction action : actions) {
            fullAuto = fullAuto.andThen(action.doAction());
        }
        return fullAuto;
    }

    public static interface AutoAction {
        Command doAction();
    }

    public static class Wait implements AutoAction {
        @Override
        public Command doAction() {
            return new WaitCommand(1.0);
        }
    }

    public static class Descore implements AutoAction {
        @Override
        public Command doAction() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'doAction'");
        }
    }
}
