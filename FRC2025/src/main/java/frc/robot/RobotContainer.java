// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.oi.OI.Bumper.LB;
import static frc.robot.oi.OI.Bumper.RB;
import static frc.robot.oi.OI.Button.B;
import static frc.robot.oi.OI.Button.BACK;
import static frc.robot.oi.OI.Button.POV_DOWN;
import static frc.robot.oi.OI.Button.POV_LEFT;
import static frc.robot.oi.OI.Button.POV_RIGHT;
import static frc.robot.oi.OI.Button.POV_UP;
import static frc.robot.oi.OI.Button.START;
import static frc.robot.oi.OI.Button.X;
import static frc.robot.oi.OI.Button.Y;
import static frc.robot.oi.OI.Trigger.LT;
import static frc.robot.oi.OI.Trigger.RT;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DescorerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.AlignmentCommands;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.manipulator.DescoreHigh;
import frc.robot.commands.manipulator.DescoreLow;
import frc.robot.oi.OI;
import frc.robot.oi.OperatorControllerOI;
import frc.robot.oi.SeperateControllerOI;
import frc.robot.subsystems.algae.AlgaeIntakeIO;
import frc.robot.subsystems.algae.AlgaeIntakeIOHardware;
import frc.robot.subsystems.algae.AlgaeIntakeIOSim;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.descorer.DescorerIO;
import frc.robot.subsystems.descorer.DescorerIOHardware;
import frc.robot.subsystems.descorer.DescorerSubsystem;
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
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.SimulationUtils;
import frc.robot.util.dashboards.DriverDash;
import frc.robot.util.dashboards.SystemsCheckDash;
import frc.robot.util.motor.MotorIOTalonFX;
import frc.robot.util.vision.QuestNav;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final QuestNav questNav = new QuestNav();

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final DescorerSubsystem descorerSubsystem;
  //private final LEDSubsystem ledSubsystem;

  private SwerveDriveSimulation driveSimulation = null;
  private IntakeSimulation intakeSimulation = null;

  private final OI oi;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Pose2d> startingPoseChooser;

  private final DriverDash driverDashboard;
  private final SystemsCheckDash systemsCheckDash;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.useSingleController) {
      oi = new OperatorControllerOI(OperatorConstants.kOperatorControllerPort);
    } else {
      oi = new SeperateControllerOI(
        OperatorConstants.kDriverControllerPort,
        OperatorConstants.kManipulatorControllerPort);
    }

    //ledSubsystem = new LEDSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem = new DriveSubsystem(
          new GyroIONavX(),
          new ModuleIOKraken(ModuleConstants.FRONT_LEFT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.FRONT_RIGHT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.BACK_LEFT_CONSTANTS),
          new ModuleIOKraken(ModuleConstants.BACK_RIGHT_CONSTANTS),
          questNav);

        visionSubsystem = new VisionSubsystem(
          driveSubsystem,
          new VisionIOPhotonVision("reef_cam_1", VisionConstants.ROBOT_TO_CAMERA_0),
          new VisionIOPhotonVision("reef_cam_2", VisionConstants.ROBOT_TO_CAMERA_1),
          new VisionIOQuestNav(questNav));

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOHardware(AlgaeIntakeConstants.ROLLER_CONFIG, AlgaeIntakeConstants.ANGLE_CONFIG));

        elevatorSubsystem = new ElevatorSubsystem(oi,
          new ElevatorIOHardware(
            new MotorIOTalonFX(ElevatorConstants.LEFT_MOTOR_ID, ElevatorConstants.LEFT_CONFIG),
            new MotorIOTalonFX(ElevatorConstants.RIGHT_MOTOR_ID, ElevatorConstants.RIGHT_CONFIG)));

        descorerSubsystem = new DescorerSubsystem(
          new DescorerIOHardware(DescorerConstants.WRIST_BASE_CONFIG, DescorerConstants.ROLLER_BASE_CONFIG));
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
          new ModuleIOSim(),
          questNav);

        visionSubsystem = new VisionSubsystem(
          driveSubsystem,
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0, driveSimulation::getSimulatedDriveTrainPose),
          new VisionIOPhotonVisionSim(
            VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1, driveSimulation::getSimulatedDriveTrainPose));
        
        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(
          new AlgaeIntakeIOSim());

        elevatorSubsystem = new ElevatorSubsystem(oi, new ElevatorIO() { });

        descorerSubsystem = new DescorerSubsystem(new DescorerIO() { });
        break;
    
      default:
        driveSubsystem = new DriveSubsystem(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          questNav);

        visionSubsystem = new VisionSubsystem(driveSubsystem, new VisionIO() {}, new VisionIO() {});

        algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new AlgaeIntakeIO() {});

        elevatorSubsystem = new ElevatorSubsystem(oi, new ElevatorIO() { });

        descorerSubsystem = new DescorerSubsystem(new DescorerIO() { });
        break;
    }

    NamedCommands.registerCommand("ElevatorFLOOR", Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR), elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL1", Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L1), elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL2", Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L2), elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL3", Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L3), elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL4", Commands.runOnce(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L4), elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorSTATION", Commands.runOnce(() -> elevatorSubsystem.moveToStation(), elevatorSubsystem));

    NamedCommands.registerCommand("StartCoralOut", Commands.runOnce(() -> elevatorSubsystem.runIntake(AutoConstants.CORAL_INTAKE_SPEED), elevatorSubsystem));
    NamedCommands.registerCommand("StopCoral", Commands.runOnce(() -> elevatorSubsystem.runIntake(0.0), elevatorSubsystem));

    NamedCommands.registerCommand("DescoreLow", new DescoreLow(descorerSubsystem));
    NamedCommands.registerCommand("DescoreHigh", new DescoreHigh(descorerSubsystem, elevatorSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();
    startingPoseChooser = new SendableChooser<>();

    autoChooser.addOption("LEFT: 2 Coral 1 Descore", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.oneDescoreTwoCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, false)));
    autoChooser.addOption("RIGHT: 2 Coral 1 Descore", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.oneDescoreTwoCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, true)));
    autoChooser.addOption("LEFT: 2 Coral", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.twoCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, false)));
    autoChooser.addOption("RIGHT: 2 Coral", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.twoCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, true)));
    autoChooser.addOption("LEFT: 3 Coral", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.threeCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, false)));
    autoChooser.addOption("RIGHT: 3 Coral", Commands.runOnce(() -> driveSubsystem.resetOdometry())
      .andThen(Autos.threeCoral(driveSubsystem, elevatorSubsystem, descorerSubsystem, visionSubsystem, true)));
    
    startingPoseChooser.setDefaultOption("Left, facing away", new Pose2d(7.75, 6.16, new Rotation2d()));
    startingPoseChooser.addOption("Left, facing towards", new Pose2d(7.75, 6.16, Rotation2d.fromDegrees(180)));
    startingPoseChooser.addOption("Right, facing away", new Pose2d(7.732, 1.897, new Rotation2d()));
    startingPoseChooser.addOption("Right, facing towards", new Pose2d(7.732, 1.897, Rotation2d.fromDegrees(180)));
    startingPoseChooser.addOption("Center, facing away", new Pose2d(3, 3, new Rotation2d(Units.degreesToRadians(90))));
    startingPoseChooser.addOption("Center, facing towards", new Pose2d(3, 3, new Rotation2d(Units.degreesToRadians(-90))));

    // Configure the trigger bindings
    configureBindings();

    driverDashboard = new DriverDash(driveSubsystem, questNav);
    systemsCheckDash = new SystemsCheckDash(driveSubsystem, elevatorSubsystem, descorerSubsystem, algaeIntakeSubsystem);

    elevatorSubsystem.initTab();
    algaeIntakeSubsystem.initTab();

    driverDashboard.initTab();
    systemsCheckDash.initTab();

    SmartDashboard.putData("Auto", autoChooser);
    SmartDashboard.putData("Starting Pose", startingPoseChooser);
  }

  public void resetElevator() {
    if(elevatorSubsystem != null) {
      elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR);
    }
  }

  private double getDriveSpeedMultiplier(double leftTriggerAxis) {
    double axis = 1 - leftTriggerAxis;
    axis *= 0.5;
    axis += 0.5;
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
    if (Constants.enableDrive) {
      driveSubsystem.setDefaultCommand(
        new JoystickDrive(driveSubsystem,
          () -> -oi.getDriveLeftY(),
          () -> -oi.getDriveLeftX(),
          () -> -oi.getDriveRightX(),
          () -> getDriveSpeedMultiplier(oi.getDriveTriggerAxis(LT))));
    }
    
         
    algaeIntakeSubsystem.setDefaultCommand(
      Commands.run(() -> algaeIntakeSubsystem.apply(0, AlgaeIntakeConstants.INTAKE_ANGLE_UP), algaeIntakeSubsystem)
    );

    elevatorSubsystem.setDefaultCommand(
      Commands.run(() -> {
        elevatorSubsystem.runIntake(0.0);
        elevatorSubsystem.setElevatorVelocity(() -> MathUtil.applyDeadband(-oi.getManipulatorLeftY(), 0.1) * 0.1);
      }, elevatorSubsystem)
    );

    descorerSubsystem.setDefaultCommand(
      Commands.run(() -> {
        descorerSubsystem.applyWristSetpoint(DescorerConstants.DESCORER_OFF_POSITION);
        descorerSubsystem.runRoller(0);
      }, descorerSubsystem));

    /* 
    ledSubsystem.setDefaultCommand(
      Commands.run(() -> {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          ledSubsystem.foreach((index) -> ledSubsystem.setSegment(index, Color.kBlue));
        } else {
          ledSubsystem.foreach((index) -> ledSubsystem.setSegment(index, Color.kRed));
        }
      }, ledSubsystem));*/
      
    
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> {}
        : () -> driveSubsystem.zeroGyro(DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero)
            : Rotation2d.kZero);

    oi.onDriveButtonPressed(Y, Commands.runOnce(resetGyro, driveSubsystem)
      .ignoringDisable(true));
    oi.onDriveButtonPressed(B,
      Commands.runOnce(() -> driveSubsystem.resetOdometry()));

    //oi.whileDriveTriggerPressed(RT,
    //  AlignmentCommands.alignToPose(driveSubsystem, () -> new Pose2d(6.0, 6.0, Rotation2d.kZero), 0.1));

    oi.whileDriveBumperPressed(RB,
      AlignmentCommands.alignToTagRight(driveSubsystem, visionSubsystem));
    oi.whileDriveBumperPressed(LB,
      AlignmentCommands.alignToTagLeft(driveSubsystem, visionSubsystem));
      

    oi.whileManipulatorBumperPressed(LB,
      Commands.run(() -> algaeIntakeSubsystem.apply(0.8, AlgaeIntakeConstants.INTAKE_ANGLE_DOWN), algaeIntakeSubsystem));
    oi.whileManipulatorBumperPressed(RB,
      Commands.run(() -> algaeIntakeSubsystem.setRollerSpeed(-0.8), algaeIntakeSubsystem));

    oi.whileManipulatorTriggerPressedFullRange(LT,
      Commands.run(() -> elevatorSubsystem.runIntake((oi.getManipulatorTriggerAxis(LT) * 0.3000)), elevatorSubsystem));
    oi.whileManipulatorTriggerPressedFullRange(RT,
      Commands.run(() -> elevatorSubsystem.runIntake((oi.getManipulatorTriggerAxis(RT) * -0.3000)), elevatorSubsystem));

    oi.onManipulatorButtonPressed(START,
      Commands.run(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.L4), elevatorSubsystem));
    oi.onManipulatorButtonPressed(BACK,
      Commands.run(() -> elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR), elevatorSubsystem));
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

    oi.whileManipulatorButtonPressed(POV_RIGHT,
      new DescoreHigh(descorerSubsystem, elevatorSubsystem));
    oi.whileManipulatorButtonPressed(POV_LEFT,
      new DescoreLow(descorerSubsystem));
    
    oi.onDriveButtonPressed(X, Commands.runOnce(() -> visionSubsystem.initializeQuestnav(), visionSubsystem));
  }

  public void updateShuffleboard() {
    elevatorSubsystem.updateTab();
    algaeIntakeSubsystem.updateTab();

    driverDashboard.updateTab();
    systemsCheckDash.updateTab();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
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

  private Pose2d lastPose;

  public void updateChoosers() {
    Pose2d startingPose = startingPoseChooser.getSelected();

    if (lastPose != startingPose) {
      Constants.INITIAL_POSITION = startingPose;
      driveSubsystem.resetOdometry();
    }

    lastPose = startingPose;
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
