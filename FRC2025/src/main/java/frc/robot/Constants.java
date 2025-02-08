// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

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
  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final boolean TUNING_MODE = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class DriveConstants {
    public static final double ODOMETRY_FREQUENCY = 0.02;

    // TODO: this
    public static final double TRACK_WIDTH = Units.inchesToMeters(28.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(28.0);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2.0, WHEEL_BASE / 2.0);
    public static final double WHEEL_RAIDUS_METERS = Units.inchesToMeters(2);
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.45;
    public static final double MAX_ACCELERATION = 4.5;
    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(720);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(1070);

    public static final Translation2d[] MODULE_OFFSETS = {
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int TURN_CURRENT_LIMIT = 25;

    private static final double degreesOffsetAll = -60;
    public static final Rotation2d FRONT_LEFT_ZERO_ROTATION = Rotation2d.fromRotations(-0.19 + 0.15 + Units.degreesToRotations(degreesOffsetAll));
    public static final Rotation2d FRONT_RIGHT_ZERO_ROTATION = Rotation2d.fromRotations(1.26 + 0.45 + Units.degreesToRotations(degreesOffsetAll));
    public static final Rotation2d BACK_LEFT_ZERO_ROTATION = Rotation2d.fromRotations(1.78 + 0.9 + 0.45 + Units.degreesToRotations(degreesOffsetAll));
    public static final Rotation2d BACK_RIGHT_ZERO_ROTATION = Rotation2d.fromRotations(-1.45 + 0.1 + Units.degreesToRotations(degreesOffsetAll));

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

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double TURN_GEAR_RATIO = 12.8;

    public static final double DRIVE_ENCODER_POSITION_FACTOR = MathConstants.TAU;
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0;

    public static final double TURN_ENCODER_POSITION_FACTOR = MathConstants.TAU;
    public static final double TURN_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0;

    public static final double DRIVE_P = 0.05;
    public static final double DRIVE_D = 0.4;

    public static final double TURN_P = 1.8;
    public static final double TURN_D = 0.055;

    public static final boolean TURN_INVERTED = false;
    public static final boolean TURN_ENCODER_INVERTED = false;

    public static final double TURN_PID_MIN_INPUT = 0;
    public static final double TURN_PID_MAX_INPUT = MathConstants.TAU;

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
                DCMotor.getNEO(1).withReduction(DRIVE_GEAR_RATIO),
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
    public static final int ROLLER_CURRENT_LIMIT = 6;
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

    public static final double ANGLE_KP = 0.13;
    public static final double ANGLE_KD = 0.0;
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
