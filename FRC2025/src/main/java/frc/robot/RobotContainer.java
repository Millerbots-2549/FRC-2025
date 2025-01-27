// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.RunAlgaeIntake;
import frc.robot.commands.StopAlgaeIntake;
import frc.robot.commands.drive.AlignToPose;
import frc.robot.commands.drive.PathfindToPose;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.subsystems.algae.AlgaeIntakeIO;
import frc.robot.subsystems.algae.AlgaeIntakeIOHardware;
import frc.robot.subsystems.algae.AlgaeIntakeIOSim;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
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
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

  private SwerveDriveSimulation driveSimulation = null;
  private IntakeSimulation intakeSimulation = null;

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

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOHardware(
            AlgaeIntakeConstants.ROLLER_CONFIG,
            AlgaeIntakeConstants.ANGLE_CONFIG));
        break;

      case SIM:
        driveSimulation = new SwerveDriveSimulation(
          DriveConstants.MAPLE_SIM_CONFIG,
          new Pose2d(3, 3, new Rotation2d()));
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
          "Algae",
          driveSimulation,
          Meters.of(0.7),
          Meters.of(0.2),
          IntakeSide.FRONT,
          1);
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
        
        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOSim());
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

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new AlgaeIntakeIO() {});
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
    
    driverController.a().onTrue(new PathfindToPose(driveSubsystem, () -> new Pose2d(new Translation2d(3, 3), Rotation2d.kZero), 0.0));
    driverController.leftBumper().onTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> intakeSimulation.startIntake(), visionSubsystem),
        new RunAlgaeIntake(algaeIntakeSubsystem)));
    driverController.leftBumper().onFalse(
      new ParallelCommandGroup(
        new RunCommand(() -> intakeSimulation.stopIntake(), visionSubsystem),
        new StopAlgaeIntake(algaeIntakeSubsystem)));
    driverController.rightBumper().onTrue(
      new InstantCommand(() -> launchAlgae(), algaeIntakeSubsystem));
    
    driverController.povUp().whileTrue(
      new RunCommand(() -> driveSubsystem.runModule(
        Rotation2d.fromDegrees(Math.sin(Timer.getTimestamp())), 0.5, 0), driveSubsystem));
    driverController.povRight().whileTrue(
      new RunCommand(() -> driveSubsystem.runModule(
        Rotation2d.fromDegrees(Math.sin(Timer.getTimestamp())), 0.5, 1), driveSubsystem));
    driverController.povLeft().whileTrue(
      new RunCommand(() -> driveSubsystem.runModule(
        Rotation2d.fromDegrees(Math.sin(Timer.getTimestamp())), 0.5,2), driveSubsystem));
    driverController.povDown().whileTrue(
      new RunCommand(() -> driveSubsystem.runModule(
        Rotation2d.fromDegrees(Math.sin(Timer.getTimestamp())), 0.5,3), driveSubsystem));
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

  public void launchAlgae() {
    if (intakeSimulation.obtainGamePieceFromIntake()) {
      SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
          driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
          new Translation2d(),
          driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
          driveSimulation.getSimulatedDriveTrainPose().getRotation(),
          0.4,
          0.9,
          Math.toRadians(70))
          .withProjectileTrajectoryDisplayCallBack(
            (poses) -> Logger.recordOutput("SuccessfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
            (poses) -> Logger.recordOutput("MissedShotsTrajectory", poses.toArray(Pose3d[]::new))));
    }
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
      "FieldSimulation/ZeroedComponentPoses", new Pose3d[] { new Pose3d(0.3, -0.35, 0.04, new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(90))) });
    Logger.recordOutput(
      "FieldSimulation/FinalComponentPoses", 
      new Pose3d[] {
        SimulationUtils.getAlgaeIntakePose(algaeIntakeSubsystem.getArmPosition().getRadians()),
        SimulationUtils.getElevatorMiddlePose((Math.sin(Timer.getTimestamp()) + 1) / 2),
        SimulationUtils.getElevatorInnerPose((Math.sin(Timer.getTimestamp()) + 1) / 2)
      });
    Logger.recordOutput(
      "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));
    Logger.recordOutput(
      "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
  }
}
