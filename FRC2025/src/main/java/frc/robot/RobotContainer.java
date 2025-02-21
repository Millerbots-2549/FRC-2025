// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.oi.OI.Button.*;
import static frc.robot.oi.OI.Bumper.*;
import static frc.robot.oi.OI.Trigger.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.oi.OI;
import frc.robot.oi.OperatorControllerOI;
import frc.robot.oi.SeperateControllerOI;
import frc.robot.subsystems.algae.AlgaeIntakeIO;
import frc.robot.subsystems.algae.AlgaeIntakeIOHardware;
import frc.robot.subsystems.algae.AlgaeIntakeIOSim;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorLevel;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.SimulationUtils;
import frc.robot.util.motor.MotorIOTalonFX;
import frc.robot.util.vision.QuestNav;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final QuestNav questNav;

  private final DriveSubsystem driveSubsystem;
  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private SwerveDriveSimulation driveSimulation = null;
  private IntakeSimulation intakeSimulation = null;

  private final OI oi;

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    questNav = new QuestNav();

    if (Constants.useSingleController) {
      oi = new OperatorControllerOI(OperatorConstants.kOperatorControllerPort);
    } else {
      oi = new SeperateControllerOI(
        OperatorConstants.kDriverControllerPort,
        OperatorConstants.kManipulatorControllerPort);
    }

    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem = new DriveSubsystem(
          new GyroIONavX(),
          new ModuleIOKraken(ModuleConstants.FRONT_LEFT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.FRONT_RIGHT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.BACK_LEFT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.BACK_RIGHT_CONSTANTS));

        visionSubsystem = new VisionSubsystem(
          driveSubsystem, new VisionIOQuestNav(questNav, new Transform3d()));

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOHardware(AlgaeIntakeConstants.ROLLER_CONFIG, AlgaeIntakeConstants.ANGLE_CONFIG));

        elevatorSubsystem = new ElevatorSubsystem(
          new ElevatorIOHardware(
            new MotorIOTalonFX(ElevatorConstants.LEFT_MOTOR_ID, ElevatorConstants.LEFT_CONFIG),
            new MotorIOTalonFX(ElevatorConstants.RIGHT_MOTOR_ID, ElevatorConstants.RIGHT_CONFIG)));
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
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());

        visionSubsystem = new VisionSubsystem(
          driveSubsystem,
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0, driveSimulation::getSimulatedDriveTrainPose),
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1, driveSimulation::getSimulatedDriveTrainPose));
        
        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOSim());

        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() { });
        break;
    
      default:
        driveSubsystem = new DriveSubsystem(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});

        visionSubsystem = new VisionSubsystem(driveSubsystem, new VisionIO() {}, new VisionIO() {});

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new AlgaeIntakeIO() {});

        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() { });
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

  private double getDriveSpeedMultiplier(double leftTriggerAxis) {
    double axis = 1 - leftTriggerAxis;
    axis *= 0.7;
    axis += 0.3;
    return axis;
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
      new JoystickDrive(driveSubsystem,
        () -> -oi.getDriveLeftY(),
        () -> -oi.getDriveLeftX(),
        () -> -oi.getDriveRightX(),
        () -> getDriveSpeedMultiplier(oi.getDriveTriggerAxis(LT))));
    
         

    algaeIntakeSubsystem.setDefaultCommand(
      Commands.run(() -> algaeIntakeSubsystem.apply(0, AlgaeIntakeConstants.INTAKE_ANGLE_UP), algaeIntakeSubsystem)
    );

    elevatorSubsystem.setDefaultCommand(
      Commands.run(() -> {
        elevatorSubsystem.runIntake(0.0);
        elevatorSubsystem.setElevatorVelocity(() -> MathUtil.applyDeadband(-oi.getManipulatorLeftY(), 0.1) * 0.1);
      }, elevatorSubsystem)
    );
    
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> {}
        : () -> driveSubsystem.zeroGyro(DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero)
            : Rotation2d.kZero);

    oi.onDriveButtonPressed(Y, Commands.runOnce(resetGyro, driveSubsystem)
      .ignoringDisable(true));

    //driverController.a().onTrue(new PathfindToPose(driveSubsystem, () -> new Pose2d(new Translation2d(3, 3), Rotation2d.kZero), 0.0));
    
    /*
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
      */

    //manipulatorController.rightBumper().whileTrue(
    //  Commands.run(() -> algaeIntakeSubsystem.apply(0.4, AlgaeIntakeConstants.INTAKE_ANGLE_DOWN), algaeIntakeSubsystem));

    /* 
    driverController.leftBumper().onTrue(
      new IntakeAlgae(algaeIntakeSubsystem).onlyWhile(() -> driverController.leftBumper().getAsBoolean()));
      */
    oi.whileManipulatorBumperPressed(LB,
      Commands.run(() -> algaeIntakeSubsystem.apply(1.0, AlgaeIntakeConstants.INTAKE_ANGLE_DOWN), algaeIntakeSubsystem));
    oi.whileManipulatorBumperPressed(LB,
      Commands.run(() -> algaeIntakeSubsystem.setRollerSpeed(-1.0), algaeIntakeSubsystem));

    oi.whileManipulatorTriggerPressed(LT,
      Commands.run(() -> elevatorSubsystem.runIntake(0.5), elevatorSubsystem));
    oi.whileManipulatorTriggerPressed(RT,
      Commands.run(() -> elevatorSubsystem.runIntake(-0.5), elevatorSubsystem));
    
    oi.onManipulatorButtonPressed(START,
      Commands.run(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L4), elevatorSubsystem));
    oi.onManipulatorButtonPressed(BACK,
      Commands.run(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L1), elevatorSubsystem));
    oi.onManipulatorButtonPressed(B,
      Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L3), elevatorSubsystem));
    oi.onManipulatorButtonPressed(X,
      Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L2), elevatorSubsystem));
    oi.onManipulatorButtonPressed(POV_UP,
      Commands.runOnce(() -> elevatorSubsystem.nextLevel(), elevatorSubsystem));
    oi.onManipulatorButtonPressed(POV_DOWN,
      Commands.runOnce(() -> elevatorSubsystem.previousLevel(), elevatorSubsystem));
    oi.onManipulatorButtonPressed(Y,
      Commands.runOnce(() -> elevatorSubsystem.moveToStation(), elevatorSubsystem));
    
      /*
    manipulatorController.a().onTrue(
      Commands.runOnce(() -> elevatorSubsystem.playMusic(), elevatorSubsystem));
    manipulatorController.b().onTrue(
      Commands.runOnce(() -> elevatorSubsystem.stopMusic(), elevatorSubsystem));
      */
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
          0.3,
          3.5,
          Math.toRadians(50))
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
    
    Logger.recordOutput("FieldPresets/ReefPositions", FieldConstants.REEF_POSITIONS);
  }
}
