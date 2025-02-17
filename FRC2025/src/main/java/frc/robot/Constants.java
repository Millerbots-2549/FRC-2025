// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.COUPLE_RATIO;
import static frc.robot.Constants.DriveConstants.DRIVE_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.DriveConstants.INVERT_LEFT;
import static frc.robot.Constants.DriveConstants.INVERT_RIGHT;
import static frc.robot.Constants.DriveConstants.TRACK_WIDTH;
import static frc.robot.Constants.DriveConstants.TURN_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.TURN_GEAR_RATIO;
import static frc.robot.Constants.DriveConstants.WHEEL_BASE;
import static frc.robot.Constants.DriveConstants.WHEEL_RADIUS;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.SimulationUtils;
import frc.robot.util.controllers.AsymmetricTrapezoidProfile;
import frc.robot.util.controllers.AsymmetricTrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Mode currentMode = RobotBase.isSimulation() ? Mode.SIM : Mode.REAL;

  //static { currentMode = Mode.REPLAY; }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final boolean TUNING_MODE = false;

  public static final boolean minimalLogging = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class DriveConstants {
    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants();

    public static final boolean INVERT_LEFT = false;
    public static final boolean INVERT_RIGHT = false;

    public static final double ODOMETRY_FREQUENCY = 0.02;

    // TODO: this
    public static final double TRACK_WIDTH = Units.inchesToMeters(28.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(28.0);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2.0, WHEEL_BASE / 2.0);
    public static final Distance WHEEL_RADIUS = Inches.of(2.167);
    public static final double WHEEL_RAIDUS_METERS = WHEEL_RADIUS.in(Meters);
    public static final double MAX_SPEED_METERS_PER_SECOND = 7.0;
    public static final double MAX_ACCELERATION = 5.0;
    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(720);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(1070);

    public static final Translation2d[] MODULE_OFFSETS = {
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int TURN_CURRENT_LIMIT = 30;

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double TURN_GEAR_RATIO = 12.8;
    public static final double COUPLE_RATIO = 1.0;

    public static final double ROBOT_MASS_KG = 74.0; // TODO: this
    public static final double ROBOT_MOI = 6.883; // TODO: this
    public static final double WHEEL_COF = 1.2; // TODO: this
    public static final RobotConfig PATH_PLANNER_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                WHEEL_RAIDUS_METERS,
                MAX_SPEED_METERS_PER_SECOND,
                WHEEL_COF,
                DCMotor.getKrakenX60(1).withReduction(DRIVE_GEAR_RATIO),
                DRIVE_CURRENT_LIMIT,
                1),
            MODULE_OFFSETS);

    public static final double DRIVE_INERTIA = 0.025;
    public static final double TURN_INERTIA = 0.004;

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(MODULE_OFFSETS)
        .withRobotMass(Kilogram.of(ROBOT_MASS_KG))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
          () -> new SwerveModuleSimulation(
            new SwerveModuleSimulationConfig(
              DCMotor.getNEO(1),
              DCMotor.getNEO(1),
              DRIVE_GEAR_RATIO,
              18.0,
              Volts.of(0.1),
              Volts.of(0.1),
              Meters.of(WHEEL_RAIDUS_METERS),
              KilogramSquareMeters.of(0.02),
              WHEEL_COF)));

    public static final double ALIGNMENT_MIN_DISTANCE = 0.5;

    public static final double ALIGNMENT_MIN_TRANSLATION_ERROR = 0.05;
    public static final double ALIGNMENT_MIN_THETA_ERROR = Units.degreesToRadians(5);
  }

  public static class ModuleConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 8;
    public static final int FRONT_RIGHT_DRIVE_ID = 11;
    public static final int BACK_LEFT_DRIVE_ID = 4;
    public static final int BACK_RIGHT_DRIVE_ID = 2;

    public static final int FRONT_LEFT_TURN_ID = 7;
    public static final int FRONT_RIGHT_TURN_ID = 10;
    public static final int BACK_LEFT_TURN_ID = 6;
    public static final int BACK_RIGHT_TURN_ID = 3;

    public static final int FRONT_LEFT_CANCODER_ID = 7;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int BACK_LEFT_CANCODER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 1;

    private static final double TURN_KP = 100;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.5;
    private static final double TURN_KS = 0.1;
    private static final double TURN_KV = 1.91;
    private static final double TURN_KA = 0.0;

    private static final double DRIVE_KP = 0.1;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV = 0.124;
    private static final double DRIVE_KA = 0.0;

    private static final Slot0Configs TURN_GAINS =
      new Slot0Configs()
        .withKP(TURN_KP)
        .withKI(TURN_KI)
        .withKD(TURN_KD)
        .withKS(TURN_KS)
        .withKV(TURN_KV)
        .withKA(TURN_KA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs()
        .withKP(DRIVE_KP)
        .withKI(DRIVE_KI)
        .withKD(DRIVE_KD)
        .withKS(DRIVE_KS)
        .withKV(DRIVE_KV)
        .withKA(DRIVE_KA);
    
    private static final ClosedLoopOutputType TURN_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement TURN_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    private static final Current SLIP_CURRENT = Amps.of(DRIVE_CURRENT_LIMIT);

    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration();
    private static final TalonFXConfiguration TURN_INITIAL_CONFIGS = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(TURN_CURRENT_LIMIT))
          .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS = new CANcoderConfiguration();

    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.025);

    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR =
      new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withCouplingGearRatio(COUPLE_RATIO)
        .withWheelRadius(WHEEL_RADIUS)
        .withSteerMotorGains(TURN_GAINS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSteerMotorClosedLoopOutput(TURN_CLOSED_LOOP_OUTPUT)
        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
        .withSlipCurrent(SLIP_CURRENT)
        .withSpeedAt12Volts(MetersPerSecond.of(4.69))
        .withDriveMotorType(DRIVE_MOTOR_TYPE)
        .withSteerMotorType(TURN_MOTOR_TYPE)
        .withFeedbackSource(STEER_FEEDBACK_TYPE)
        .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
        .withSteerMotorInitialConfigs(TURN_INITIAL_CONFIGS)
        .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
        .withSteerInertia(STEER_INERTIA)
        .withDriveInertia(DRIVE_INERTIA)
        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;
    private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.0);
    private static final Distance FRONT_LEFT_X_POS = Inches.of(WHEEL_BASE / 2);
    private static final Distance FRONT_LEFT_Y_POS = Inches.of(TRACK_WIDTH / 2);

    private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(0.0);
    private static final Distance FRONT_RIGHT_X_POS = Inches.of(WHEEL_BASE / 2);
    private static final Distance FRONT_RIGHT_Y_POS = Inches.of(-TRACK_WIDTH / 2);

    private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_LEFT_ENCODER_INVERTED = false;
    private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.0);
    private static final Distance BACK_LEFT_X_POS = Inches.of(-WHEEL_BASE / 2);
    private static final Distance BACK_LEFT_Y_POS = Inches.of(TRACK_WIDTH / 2);

    private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(0.0);
    private static final Distance BACK_RIGHT_X_POS = Inches.of(-WHEEL_BASE / 2);
    private static final Distance BACK_RIGHT_Y_POS = Inches.of(-TRACK_WIDTH / 2);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT_CONSTANTS =
      CONSTANT_CREATOR.createModuleConstants(
        FRONT_LEFT_DRIVE_ID,
        FRONT_LEFT_TURN_ID,
        FRONT_LEFT_CANCODER_ID,
        FRONT_LEFT_ENCODER_OFFSET,
        FRONT_LEFT_X_POS,
        FRONT_LEFT_Y_POS,
        INVERT_LEFT,
        FRONT_LEFT_STEER_MOTOR_INVERTED,
        FRONT_LEFT_ENCODER_INVERTED);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT_CONSTANTS =
      CONSTANT_CREATOR.createModuleConstants(
        FRONT_RIGHT_DRIVE_ID,
        FRONT_RIGHT_TURN_ID,
        FRONT_RIGHT_CANCODER_ID,
        FRONT_RIGHT_ENCODER_OFFSET,
        FRONT_RIGHT_X_POS,
        FRONT_RIGHT_Y_POS,
        INVERT_RIGHT,
        FRONT_RIGHT_STEER_MOTOR_INVERTED,
        FRONT_RIGHT_ENCODER_INVERTED);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT_CONSTANTS =
      CONSTANT_CREATOR.createModuleConstants(
        BACK_LEFT_DRIVE_ID,
        BACK_LEFT_TURN_ID,
        BACK_LEFT_CANCODER_ID,
        BACK_LEFT_ENCODER_OFFSET,
        BACK_LEFT_X_POS,
        BACK_LEFT_Y_POS,
        INVERT_LEFT,
        BACK_LEFT_STEER_MOTOR_INVERTED,
        BACK_LEFT_ENCODER_INVERTED);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT_CONSTANTS =
      CONSTANT_CREATOR.createModuleConstants(
        BACK_RIGHT_DRIVE_ID,
        BACK_RIGHT_TURN_ID,
        BACK_RIGHT_CANCODER_ID,
        BACK_RIGHT_ENCODER_OFFSET,
        BACK_RIGHT_X_POS,
        BACK_RIGHT_Y_POS,
        INVERT_RIGHT,
        BACK_RIGHT_STEER_MOTOR_INVERTED,
        BACK_RIGHT_ENCODER_INVERTED);
  }

  public static class VisionConstants {
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static String CAMERA_0_NAME = "camera_0";
    public static String CAMERA_1_NAME = "camera_1";

    public static Transform3d ROBOT_TO_CAMERA_0 = new Transform3d(0.0, 0.2, 0.2, new Rotation3d(0.0, -0.4, Math.PI / 2));
    public static Transform3d ROBOT_TO_CAMERA_1 = new Transform3d(0.0, -0.2, 0.2, new Rotation3d(0.0, -0.4, Math.PI * 1.5));

    public static double MAX_AMBIGUITY = 0.3;
    public static double MAX_Z_ERROR = 0.75;

    public static double LINEAR_STD_DEV_BASELINE = 0.02; 
    public static double ANGULAR_STD_DEV_BASELINE = 0.06;

    public static double[] CAMERA_STD_DEV_FACTORS = new double[] {
        1.0,
        1.0
    };

    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5;
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY;
  }

  public static class AlgaeIntakeConstants {
    public static final double ANGLE_GEAR_RATIO = 5;
    public static final double ROLLER_GEAR_RATIO = 1;

    public static final int ROLLER_MOTOR_ID = 21;
    public static final int ROLLER_CURRENT_LIMIT = 5;
    public static final double ROLLER_ENCODER_POSITION_FACTOR = MathConstants.TAU;
    public static final double ROLLER_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0;

    public static final int ANGLE_MOTOR_ID = 16;
    public static final int ANGLE_CURRENT_LIMIT = 30;
    public static final double ANGLE_ENCODER_POSITION_FACTOR = MathConstants.TAU * ANGLE_GEAR_RATIO;
    public static final double ANGLE_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0;

    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-90);

    public static final double ROLLER_KP = 0.005;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KS = 0.0;
    public static final double ROLLER_KV = 0.0789;
    public static final double ROLLER_SIM_KP = 1.0;
    public static final double ROLLER_SIM_KD = 0.0;
    public static final double ROLLER_SIM_KS = 0.0;
    public static final double ROLLER_SIM_KV = 0.0;

    public static final double ANGLE_KP = 0.09;
    public static final double ANGLE_KD = 0.01;
    public static final double ANGLE_KS = 0.23;
    public static final double ANGLE_KG = 0.05;
    public static final double ANGLE_KV = 1.15;
    public static final double ANGLE_KA = 0.081;
    public static final double ANGLE_SIM_KP = 2.0;
    public static final double ANGLE_SIM_KD = 0.0;
    public static final double ANGLE_PID_MIN_INPUT = 0.0;
    public static final double ANGLE_PID_MAX_INPUT = MathConstants.TAU;

    public static final SparkBaseConfig ROLLER_BASE_CONFIG = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ROLLER_CURRENT_LIMIT)
        .voltageCompensation(12.0)
        .apply(
            new EncoderConfig()
                .positionConversionFactor(ROLLER_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ROLLER_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2))
        .apply(
            new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ROLLER_KP, 0.0, ROLLER_KD, 0.0))
        .apply(
            new SignalsConfig()
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int)(1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20));

    public static final SparkBaseConfig ANGLE_BASE_CONFIG = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ANGLE_CURRENT_LIMIT)
        .voltageCompensation(12.0)
        .apply(
            new EncoderConfig()
                .positionConversionFactor(ANGLE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ANGLE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2))
        .apply(
            new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(ANGLE_PID_MIN_INPUT, ANGLE_PID_MAX_INPUT)
                .pidf(ANGLE_KP, 0.0, ANGLE_KD, 0.0))
        .apply(
            new SignalsConfig()
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int)(1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20));

    public record RollerConfig(
      int ID,
      boolean invert) {};
    public record AngleConfig(
      int ID,
      boolean invert) {};

    public static final RollerConfig ROLLER_CONFIG = 
      new RollerConfig(ROLLER_MOTOR_ID, false);
    public static final AngleConfig ANGLE_CONFIG = 
      new AngleConfig(ANGLE_MOTOR_ID, false);

    public static final double ROLLER_MAX_SPEED = 1;
    public static final Rotation2d INTAKE_ANGLE_DOWN = new Rotation2d(0.45 * ANGLE_GEAR_RATIO);
    public static final Rotation2d INTAKE_ANGLE_UP = new Rotation2d(0.2 * ANGLE_GEAR_RATIO);

    public static final double INTAKE_ANGLE_TOLERANCE = 10;

    public static final double ANGLE_MAX_VELOCITY = 5.0;
    public static final double ANGLE_MAX_NEGATIVE_ACCEL = 14.0;
    public static final double ANGLE_MAX_POSITIVE_ACCEL = 6.0;

    public static final Constraints POSITIVE_CONSTRAINTS =
      new AsymmetricTrapezoidProfile.Constraints(
        ANGLE_MAX_VELOCITY,
        ANGLE_MAX_NEGATIVE_ACCEL,
        ANGLE_MAX_POSITIVE_ACCEL);
    
    public static final Constraints SLOW_CONSTRAINTS =
      new AsymmetricTrapezoidProfile.Constraints(
        ANGLE_MAX_VELOCITY,
        ANGLE_MAX_POSITIVE_ACCEL,
        ANGLE_MAX_POSITIVE_ACCEL);
    
    public static final Constraints FAST_CONSTRAINTS =
      new AsymmetricTrapezoidProfile.Constraints(
        ANGLE_MAX_VELOCITY,
        ANGLE_MAX_NEGATIVE_ACCEL,
        ANGLE_MAX_NEGATIVE_ACCEL);

    public static final double ANGLE_FORWARD_SOFT_LIMIT = 4.13f;
    public static final double ANGLE_REVERSE_SOFT_LIMIT = -1.05f;
  }

  public static class ElevatorConstants {
    public static final double ELEVATOR_KP = 0.005;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    public static final double ELEVATOR_KS = 0.0;
    public static final double ELEVATOR_KV = 0.0;
    public static final double ELEVATOR_KA = 0.0;
    public static final double ELEVATOR_KG = 0.0;

    public static final int LEFT_MOTOR_ID = 2;
    public static final int RIGHT_MOTOR_ID = 3;
    public static final double CURRENT_LIMIT = 80;
    public static final double MOTOR_TO_HEIGHT_RATIO = 1;
    public static final double CURRENT_UPPER_BOUND = 100;
    public static final TalonFXConfiguration LEFT_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs().withStatorCurrentLimit(CURRENT_LIMIT))
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
      .withSlot0(
        new Slot0Configs().withKP(ELEVATOR_KP).withKI(ELEVATOR_KI).withKD(ELEVATOR_KD)
          .withKS(ELEVATOR_KS).withKV(ELEVATOR_KV).withKA(ELEVATOR_KA)
          .withGravityType(GravityTypeValue.Elevator_Static).withKG(ELEVATOR_KG))
      .withClosedLoopRamps(
        new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.02)
          .withTorqueClosedLoopRampPeriod(0.02)
          .withVoltageClosedLoopRampPeriod(0.02));
    public static final TalonFXConfiguration RIGHT_CONFIG = LEFT_CONFIG;
  }

  public static class FieldConstants {
    public static final Translation2d REEF_CENTER = new Translation2d(4.485, 4);
    public static final double REEF_RADIUS = 2.37 / 2.0;
    public static final Pose2d[][] REEF_POSITIONS = SimulationUtils.generateReefPoses(REEF_CENTER, REEF_RADIUS, DriveConstants.WHEEL_BASE / 2, new Translation2d(0, 0), Rotation2d.fromDegrees(-90));
  }

  public static class MathConstants {
    public static final double PI = Math.PI;
    public static final double TAU = 2 * Math.PI;
  }

  public static final String FAILED_CRC_BYTES = "32 32 32 3f fe 64 64 64 7f fc c8 c8 c8 ff f9 91 91 91 ff f3 23 23 23 ff e6 46 46 47 ff cc 8c 8c 8f ff 99 19 19 1f ff 32 32 32 3f fe 64 64 64 7f fc c8 c8 c8 ff f9 91 91 91 ff f3 23 23 23 ff e6 46 46 47 ff cc 8c 8c 8f ff 99 19 19 1f ff 32 32 32 3f fe 64 64 64 7f fc c8 c8 c8 ff f9 91 91 91 ff f3 23 23 23 ff e6 46 46 47 ff cc 8c";
}
