// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.descorer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.motor.ArmFeedForward;

/** Add your docs here. */
public class DescorerIOHardware implements DescorerIO {
    /** This is the spark max used to turn the roller */
    protected final SparkMax wristMotor;
    private final RelativeEncoder wristEncoder;

    private final SparkBaseConfig wristConfig;

    private final ProfiledPIDController wristPositionController = new ProfiledPIDController(0, 0, 0, null);
    private final Constraints wristConstraints = new Constraints(0, 0);
    private final ArmFeedForward wristFeedForward = new ArmFeedForward(
        0, 0, 0, 0);

    private Rotation2d setpoint;

    public DescorerIOHardware() {
        wristMotor = new SparkMax(0, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();

        wristConfig = new SparkMaxConfig();
    }
}
