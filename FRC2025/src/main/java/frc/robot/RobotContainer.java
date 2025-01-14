// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOSparkSim;
import frc.robot.subsystems.drive.OdometryThread;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem driveSubsystem;

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
        break;
    
      default:
        driveSubsystem = new DriveSubsystem(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          null);
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
    driveSubsystem.setDefaultCommand(
      new TeleopDrive(driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX()));
    
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
