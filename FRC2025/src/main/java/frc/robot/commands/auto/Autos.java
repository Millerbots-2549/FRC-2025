// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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

    public static boolean rightSide = false;

    public static void setRightSide(boolean rightSide) {
        Autos.rightSide = rightSide;
    }

    public static boolean getRightSide() {
        return Autos.rightSide;
    }

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

    private static final Command mirrorAuto(BooleanSupplier supplier, PathPlannerPath... paths) {
        return Commands.runOnce(() -> {
            if(supplier.getAsBoolean()) {
                for(PathPlannerPath path : paths) {
                    path = path.mirrorPath();
                }
            }
        });
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
        return new DescoreHigh(descorerSubsystem, elevatorSubsystem).withTimeout(3.0);
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
            left ? alignReefLeft(level, 1.2, driveSubsystem, elevatorSubsystem, visionSubsystem)
            : alignReefRight(level, 1.2, driveSubsystem, elevatorSubsystem, visionSubsystem),
            outtake(elevatorSubsystem),
            stopIntake(elevatorSubsystem),
            lowerElevator(elevatorSubsystem)
        );
    }

    /**
     * Runs a full cycle
     * @param stationPath The path to the coral station
     * @param level The level to score
     * @param left If the robot should align to the left branch
     * @param alignTime The amount of time in seconds that the robot should auto align
     * @param driveSubsystem The drive subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param visionSubsystem The vision subsystem
     * @return A command to run a full cycle
     */
    private static final Command runCycle(Command stationPath, ElevatorLevel level, boolean left, double alignTime, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        return new SequentialCommandGroup(
            stationPath,
            left ? alignReefLeft(level, alignTime, driveSubsystem, elevatorSubsystem, visionSubsystem)
            : alignReefRight(level, alignTime, driveSubsystem, elevatorSubsystem, visionSubsystem),
            outtake(elevatorSubsystem),
            stopIntake(elevatorSubsystem),
            lowerElevator(elevatorSubsystem)
        );
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
    private static final Command runCycleNoOuttake(Command stationPath, ElevatorLevel level, boolean left, double alignTime, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        return new SequentialCommandGroup(
            stationPath,
            left ? alignReefLeft(level, alignTime, driveSubsystem, elevatorSubsystem, visionSubsystem)
            : alignReefRight(level, alignTime, driveSubsystem, elevatorSubsystem, visionSubsystem),
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

    public static final Command oneDescoreTwoCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2, part3;
        try {
            part1 = PathPlannerPath.fromPathFile("1D3C Part 1");
            part2 = PathPlannerPath.fromPathFile("1D3C Part 2");
            part3 = PathPlannerPath.fromPathFile("1D3C Part 3");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
                part3 = part3.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                descoreHigh(descorerSubsystem, elevatorSubsystem),
                AutoBuilder.followPath(part2),
                alignReefLeft(ElevatorLevel.L3, 1.6, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                stopIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part3, 2.0, 5.8, elevatorSubsystem),
                    ElevatorLevel.L3, false, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command threeCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2, part3;
        try {
            part1 = PathPlannerPath.fromPathFile("0D3C Part 1");
            part2 = PathPlannerPath.fromPathFile("0D3C Part 2");
            part3 = PathPlannerPath.fromPathFile("0D3C Part 3");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
                part3 = part3.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                alignReefRight(ElevatorLevel.L2, 1.0, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem).withDeadline(new WaitCommand(0.4)),
                slowIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part2, 0.9, 4.0, elevatorSubsystem),
                    ElevatorLevel.L2, true, 1.35, driveSubsystem, elevatorSubsystem, visionSubsystem),
                slowIntake(elevatorSubsystem),
                runCycleNoOuttake(followStationPath(part3, 0.8, 3.6, elevatorSubsystem),
                    ElevatorLevel.L2, false, 1.5, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command twoCoral(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, VisionSubsystem visionSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2;
        try {
            part1 = PathPlannerPath.fromPathFile("0D2C Part 1");
            part2 = PathPlannerPath.fromPathFile("0D2C Part 2");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                alignReefRight(ElevatorLevel.L2, 2.8, driveSubsystem, elevatorSubsystem, visionSubsystem),
                outtake(elevatorSubsystem),
                slowIntake(elevatorSubsystem),
                lowerElevator(elevatorSubsystem),
                runCycle(followStationPath(part2, 0.9, 4.5, elevatorSubsystem),
                    ElevatorLevel.L2, true, 2.8, driveSubsystem, elevatorSubsystem, visionSubsystem)
            );
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command twoDescore(DriveSubsystem driveSubsystem, DescorerSubsystem descorerSubsystem, ElevatorSubsystem elevatorSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2, part3;
        try {
            part1 = PathPlannerPath.fromPathFile("2D0C Part 1");
            part2 = PathPlannerPath.fromPathFile("2D0C Part 2");
            part3 = PathPlannerPath.fromPathFile("2D0C Part 3");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
                part3 = part3.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                descoreHigh(descorerSubsystem, elevatorSubsystem),
                AutoBuilder.followPath(part2),
                descoreLow(descorerSubsystem),
                followStationPath(part3, 1.3, 3.1, elevatorSubsystem),
                stopIntake(elevatorSubsystem)
            );
        } catch (Exception e) {
            System.out.println("Failed to create 2D0C auto");
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command threeDescore(DriveSubsystem driveSubsystem, DescorerSubsystem descorerSubsystem, ElevatorSubsystem elevatorSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2, part3, part4;
        try {
            part1 = PathPlannerPath.fromPathFile("3D0C Part 1");
            part2 = PathPlannerPath.fromPathFile("3D0C Part 2");
            part3 = PathPlannerPath.fromPathFile("3D0C Part 3");
            part4 = PathPlannerPath.fromPathFile("3D0C Part 4");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
                part3 = part3.mirrorPath();
                part4 = part4.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                descoreLow(descorerSubsystem),
                AutoBuilder.followPath(part2),
                descoreHigh(descorerSubsystem, elevatorSubsystem),
                AutoBuilder.followPath(part3),
                descoreLow(descorerSubsystem),
                followStationPath(part4, 0.4, 2.1, elevatorSubsystem),
                stopIntake(elevatorSubsystem)
            );
        } catch (Exception e) {
            System.out.println("Failed to create 3D0C auto");
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command fourDescore(DriveSubsystem driveSubsystem, DescorerSubsystem descorerSubsystem, ElevatorSubsystem elevatorSubsystem, boolean rightSide) {
        PathPlannerPath part1, part2, part3, part4, part5;
        try {
            part1 = PathPlannerPath.fromPathFile("4D0C Part 1");
            part2 = PathPlannerPath.fromPathFile("4D0C Part 2");
            part3 = PathPlannerPath.fromPathFile("4D0C Part 3");
            part4 = PathPlannerPath.fromPathFile("4D0C Part 4");
            part5 = PathPlannerPath.fromPathFile("4D0C Part 5");
            if(rightSide) {
                part1 = part1.mirrorPath();
                part2 = part2.mirrorPath();
                part3 = part3.mirrorPath();
                part4 = part4.mirrorPath();
                part5 = part5.mirrorPath();
            }
            return new SequentialCommandGroup(
                PathfindingCommands.pathfindThenFollow(part1),
                descoreLow(descorerSubsystem),
                AutoBuilder.followPath(part2),
                descoreHigh(descorerSubsystem, elevatorSubsystem),
                AutoBuilder.followPath(part3),
                descoreLow(descorerSubsystem),
                AutoBuilder.followPath(part4),
                descoreHigh(descorerSubsystem, elevatorSubsystem),
                followStationPath(part5, 0.9, 3.0, elevatorSubsystem),
                stopIntake(elevatorSubsystem)
            );
        } catch (Exception e) {
            System.out.println("Failed to create 4D0C auto");
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static final Command leave(DriveSubsystem driveSubsystem) {
        return new SequentialCommandGroup(
            PathfindingCommands.pathfindToPoint(new Pose2d(Constants.INITIAL_POSITION.getX() - 1.0, Constants.INITIAL_POSITION.getY(), Constants.INITIAL_POSITION.getRotation()))
        );
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
