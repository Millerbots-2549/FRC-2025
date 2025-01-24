// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOSparkSim;
import frc.robot.subsystems.drive.OdometryThread;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.SimulationUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private SwerveDriveSimulation driveSimulation = null;

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem = new DriveSubsystem(
          new GyroIONavX(),
          new ModuleIOSpark(SparkModuleConstants.frontLeft),
          new ModuleIOSpark(SparkModuleConstants.frontRight),
          new ModuleIOSpark(SparkModuleConstants.backLeft),
          new ModuleIOSpark(SparkModuleConstants.backRight),
          OdometryThread.getInstance());

        visionSubsystem = new VisionSubsystem(
          driveSubsystem,
          new VisionIOLimelight(VisionConstants.CAMERA_0_NAME, driveSubsystem::getRotation));
        break;

      case SIM:
        driveSimulation = new SwerveDriveSimulation(
          DriveConstants.MAPLE_SIM_CONFIG,
          new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveSubsystem = new DriveSubsystem(
          new GyroIOSim(driveSimulation.getGyroSimulation()),
          new ModuleIOSparkSim(driveSimulation.getModules()[0]),
          new ModuleIOSparkSim(driveSimulation.getModules()[1]),
          new ModuleIOSparkSim(driveSimulation.getModules()[2]),
          new ModuleIOSparkSim(driveSimulation.getModules()[3]),
          null);

        visionSubsystem = new VisionSubsystem(
          driveSubsystem,
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0, driveSimulation::getSimulatedDriveTrainPose),
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1, driveSimulation::getSimulatedDriveTrainPose));
        break;
    
      default:
        driveSubsystem = new DriveSubsystem(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          null);

        visionSubsystem = new VisionSubsystem(driveSubsystem, new VisionIO() {}, new VisionIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
      "Drive Wheel Radius Characterization", CharacterizationCommands.wheelRadiusCharacterization(driveSubsystem));
    autoChooser.addOption(
      "Drive Feed Forward Characterization", CharacterizationCommands.feedforwardCharacterization(driveSubsystem));
    autoChooser.addOption(
      "Forward Drive SysId (Quasistatic)", driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
      "Reverse Drive SysId (Quasistatic)", driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
      "Forward Drive SysId (Dynamic)", driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
      "Reverse Drive SysId (Dynamic)", driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
    driveSubsystem.setDefaultCommand(
      new TeleopDrive(driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX()));
    */

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> driveSubsystem.setPose(driveSimulation
                        .getSimulatedDriveTrainPose())
            : () -> driveSubsystem.setPose(new Pose2d(
                        driveSubsystem.getPose().getTranslation(),
                        DriverStation.getAlliance().isPresent()
                            ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                                ? new Rotation2d(Math.PI)
                                : new Rotation2d())
                            : new Rotation2d()));

    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                        resetGyro,
                    driveSubsystem)
                .ignoringDisable(true));
    
    driverController.a().onTrue(new PathfindToPose(driveSubsystem, () -> new Pose2d(new Translation2d(3, 3), Rotation2d.kZero), 2.0, () -> driveSubsystem.getCurrentPath()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }

  public void resetSimulatedField() {
    if (Constants.currentMode != Mode.SIM) {
      return;
    }

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void sendSimulatedFieldToAdvantageScope() {
    if (Constants.currentMode != Mode.SIM) {
      return;
    }

    Logger.recordOutput(
      "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
      "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));
    Logger.recordOutput(
      "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
  }
}
