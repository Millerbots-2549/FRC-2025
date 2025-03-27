// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickDrive extends Command {
  private final DoubleSupplier driverXInputSupplier, driverYInputSupplier, driverRotationalInputSupplier,
    speedMultiplierSupplier;
  private final DriveSubsystem driveSubsystem;

  private final Timer previousRotationalInputTimer;
  private final PIDController chassisRotationController;
  private Rotation2d rotationMaintenanceSetpoint;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(
      DriveSubsystem driveSubsystem,
      DoubleSupplier driverXInputSupplier,
      DoubleSupplier driverYInputSupplier,
      DoubleSupplier driverRotationalInputSupplier,
      DoubleSupplier speedMultiplierSupplier) {
    this.driverXInputSupplier = driverXInputSupplier;
    this.driverYInputSupplier = driverYInputSupplier;
    this.driverRotationalInputSupplier = driverRotationalInputSupplier;
    this.speedMultiplierSupplier = speedMultiplierSupplier;
    
    this.previousRotationalInputTimer = new Timer();
    this.driveSubsystem = driveSubsystem;
    chassisRotationController = new PIDController(0.05, 0, 0.0); // fill your pid
    chassisRotationController.enableContinuousInput(0, Math.toRadians(360));
    super.addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousRotationalInputTimer.start();
    rotationMaintenanceSetpoint = driveSubsystem.getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d linearVelocity = getVelocityFromJoysticks(
      driverXInputSupplier.getAsDouble() * speedMultiplierSupplier.getAsDouble(),
      driverYInputSupplier.getAsDouble() * speedMultiplierSupplier.getAsDouble());

    double rot = MathUtil.applyDeadband(
      driverRotationalInputSupplier.getAsDouble() * speedMultiplierSupplier.getAsDouble(),
      OperatorConstants.DEADBAND);

    rot = Math.copySign(rot * rot, rot);

    if (rot != 0 || driveSubsystem.timeSinceGyroZeroed.get() < 0.5) previousRotationalInputTimer.reset();

    /* no rotation input for 0.5 seconds, maintain current rotation */
    if (previousRotationalInputTimer.hasElapsed(0.15)) {
      rot = chassisRotationController.calculate(
        driveSubsystem.getRotation().getRadians(),
        rotationMaintenanceSetpoint.getRadians());
    }
    /* there has been a rotation input within 0.5 seconds, reset rotation maintenance setpoint */
    else {
      rotationMaintenanceSetpoint = driveSubsystem.getRotation();
    }

    ChassisSpeeds speeds = new ChassisSpeeds(
      linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec() * 1.2,
      linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec() * 1.2,
      rot * driveSubsystem.getMaxAngularSpeedRadPerSec());
    boolean isFlipped = DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Red;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      isFlipped ? driveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
      : driveSubsystem.getRotation());
    driveSubsystem.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getVelocityFromJoysticks(double x, double y) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), OperatorConstants.DEADBAND);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    magnitude = magnitude * magnitude;

    return new Pose2d(new Translation2d(), direction)
      .transformBy(new Transform2d(magnitude, 0.0, new Rotation2d()))
      .getTranslation();
  }
}
