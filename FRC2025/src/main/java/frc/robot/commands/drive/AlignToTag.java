// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MathConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@SuppressWarnings("unused")
public class AlignToTag extends Command {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private final double wantedOffset;
  private final double wantedDistance;
  private double wantedRotation;
  private final int cameraIndex;
  
  private final PIDController offsetController = new PIDController(3.4, 0.0, 0.02);
  private final PIDController distanceController = new PIDController(4.4, 0.0, 0.02);
  private final PIDController rotationController = new PIDController(3.0, 0.0, 0.0);

  private static final Map<Integer, Double> tagRotMap = new HashMap<>();
  static {
    tagRotMap.put(6, Units.degreesToRadians(210));
    tagRotMap.put(7, Units.degreesToRadians(270));
    tagRotMap.put(8, Units.degreesToRadians(330));
    tagRotMap.put(9, Units.degreesToRadians(30));
    tagRotMap.put(10, Units.degreesToRadians(90));
    tagRotMap.put(11, Units.degreesToRadians(150));

    tagRotMap.put(19, Units.degreesToRadians(210));
    tagRotMap.put(18, Units.degreesToRadians(270));
    tagRotMap.put(17, Units.degreesToRadians(330));
    tagRotMap.put(22, Units.degreesToRadians(30));
    tagRotMap.put(21, Units.degreesToRadians(90));
    tagRotMap.put(20, Units.degreesToRadians(150));
  }

  private final double maxSpeed = 2.1516;

  private boolean finish = false;

  private boolean aligned = false;

  /** Creates a new AlignToTag. */
  public AlignToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
                    double wantedOffset, double wantedDistance, int cameraIndex) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.wantedOffset = wantedOffset;
    this.wantedDistance = wantedDistance;
    this.cameraIndex = cameraIndex;

    this.finish = false;

    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    offsetController.setTolerance(0.05);
    distanceController.setTolerance(0.05);
    rotationController.setTolerance(0.005);

    rotationController.enableContinuousInput(0, MathConstants.TAU);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tagRotMap.containsKey(visionSubsystem.getTargetID(cameraIndex))) {
      wantedRotation = MathUtil.inputModulus(tagRotMap.get(
        visionSubsystem.getTargetID(cameraIndex)), 0, MathConstants.TAU);
    }

    Pose3d currentPose = visionSubsystem.getTargetPose(cameraIndex);
    double currentOffset = currentPose.getY();
    double currentDistance = currentPose.getX();
    double currentRotation = MathUtil.inputModulus(driveSubsystem.getRotation().getRadians(), 0, MathConstants.TAU);

    double offsetOutput = 0.0;
    if (currentDistance < 0.5) {
      offsetOutput = MathUtil.clamp(offsetController.calculate(currentOffset, wantedOffset), -maxSpeed, maxSpeed);
    } else {
      offsetOutput = MathUtil.clamp(offsetController.calculate(currentOffset, 0.0), -maxSpeed, maxSpeed);
    }
    double distanceOutput = MathUtil.clamp(distanceController.calculate(currentDistance, wantedDistance), -maxSpeed, maxSpeed);
    double rotationOutput = rotationController.calculate(currentRotation, wantedRotation);

    if (visionSubsystem.getTargetID(cameraIndex) <= 0 || !tagRotMap.containsKey(visionSubsystem.getTargetID(cameraIndex))) {
      driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
    } else {
      if (Math.abs(distanceOutput) < 0.5) {
        distanceOutput = 0;
      }
      driveSubsystem.runVelocity(new ChassisSpeeds(-offsetOutput, distanceOutput, rotationOutput));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
