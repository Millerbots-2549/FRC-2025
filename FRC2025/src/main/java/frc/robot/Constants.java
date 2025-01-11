// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Mode currentMode = Mode.SIM;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double ODOMETRY_FREQUENCY = 0.02;

    // TODO: this
    public static final double WHEEL_RAIDUS_METERS = 0;
    public static final double MAX_SPEED_METERS_PER_SECOND = 0;

    // TODO: this
    public static final Translation2d[] MODULE_OFFSETS = {};

    public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int TURN_CURRENT_LIMIT = 25;

    public static final Rotation2d FRONT_LEFT_ZERO_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d FRONT_RIGHT_ZERO_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d BACK_LEFT_ZERO_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d BACK_RIGHT_ZERO_ROTATION = Rotation2d.fromDegrees(0);

    // TODO: this
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int BACK_LEFT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_DRIVE_ID = 0;

    // TODO: this
    public static final int FRONT_LEFT_TURN_ID = 0;
    public static final int FRONT_RIGHT_TURN_ID = 0;
    public static final int BACK_LEFT_TURN_ID = 0;
    public static final int BACK_RIGHT_TURN_ID = 0;

    //TODO: this
    public static final int FRONT_LEFT_CANCODER_ID = 0;
    public static final int FRONT_RIGHT_CANCODER_ID = 0;
    public static final int BACK_LEFT_CANCODER_ID = 0;
    public static final int BACK_RIGHT_CANCODER_ID = 0;

    public static final double DRIVE_MOTOR_REDUCTION = 0; // TODO: this

    public static final double DRIVE_ENCODER_POSITION_FACTOR = MathConstants.TAU / DRIVE_MOTOR_REDUCTION;
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0 / DRIVE_MOTOR_REDUCTION;

    public static final double TURN_ENCODER_POSITION_FACTOR = MathConstants.TAU;
    public static final double TURN_ENCODER_VELOCITY_FACTOR = MathConstants.TAU / 60.0;

    // TODO: tune these
    public static final double DRIVE_P = 0;
    public static final double DRIVE_D = 0;

    // TODO: tune these
    public static final double TURN_P = 0;
    public static final double TURN_D = 0;

    // TODO: this
    public static final boolean TURN_INVERTED = false;
    public static final boolean TURN_ENCODER_INVERTED = false;

    public static final double TURN_PID_MIN_INPUT = 0;
    public static final double TURN_PID_MAX_INPUT = MathConstants.TAU;

    public static final double ROBOT_MASS_KG = 0.0; // TODO: this
    public static final double ROBOT_MOI = 0.0; // TODO: this
    public static final double WHEEL_COF = 0.0; // TODO: this
    public static final RobotConfig PATH_PLANNER_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                WHEEL_RAIDUS_METERS,
                MAX_SPEED_METERS_PER_SECOND,
                WHEEL_COF,
                DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
                DRIVE_CURRENT_LIMIT,
                1),
            MODULE_OFFSETS);
  }

  public static class MathConstants {
    public static final double PI = Math.PI;
    public static final double TAU = 2 * Math.PI;
  }
}