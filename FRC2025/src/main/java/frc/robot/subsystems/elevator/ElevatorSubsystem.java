// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi.OI;
import frc.robot.util.DashboardPublisher;

public class ElevatorSubsystem extends SubsystemBase implements DashboardPublisher {
  private final OI oi;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int currentLevel = 0;
  private double currentPositionSetpoint = 0;
  private Timer isDownTimer = new Timer();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(OI oi, ElevatorIO io) {
    this.oi = oi;
    this.io = io;

    isDownTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SmartDashboard.putNumber("Elevator Height", inputs.heightMeters);
    SmartDashboard.putNumber("Elevator Setpoint", inputs.heightSetpointMeters);
    SmartDashboard.putNumber("Elevator Velocity", inputs.velocityMetersPerSecond);
    SmartDashboard.putBoolean("Elevator Connected", inputs.elevatorConnected);
    SmartDashboard.putBoolean("Elevator Out of Bounds", inputs.outOfBounds);

    SmartDashboard.putNumber("L1 Setpoint", ElevatorLevel.L1.height + heightOffsets[0]);
    SmartDashboard.putNumber("L2 Setpoint", ElevatorLevel.L2.height + heightOffsets[1]);
    SmartDashboard.putNumber("L3 Setpoint", ElevatorLevel.L3.height + heightOffsets[2]);
    SmartDashboard.putNumber("L4 Setpoint", ElevatorLevel.L4.height + heightOffsets[3]);

    if (currentLevel == 0) {
      if (isDownTimer.get() < 0.5) {
        oi.setDriveRumble(RumbleType.kBothRumble, 0.5);
      } else {
        oi.setDriveRumble(RumbleType.kBothRumble, 0.0);
      }
    } else {
      isDownTimer.reset();
      oi.setDriveRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  public void setElevatorPosition(double position) {
    currentPositionSetpoint = position;
    io.applySetpointMeters(position);
  }

  public void setElevatorVelocity(DoubleSupplier velocity) {
    setElevatorPosition(currentPositionSetpoint + velocity.getAsDouble());
  }

  public void moveToStation() {
    setElevatorPosition(7.4);
    currentLevel = 2;
  }

  public void moveToLevel(ElevatorLevel level) {
    //setElevatorPosition(level.height + (currentLevel >= 1 ? heightOffsets[currentLevel - 1] : 0.0));
    setElevatorPosition(level.height);
    currentLevel = level.ordinal();
  }

  public void nextLevel() {
    if (currentLevel >= 4) return;

    moveToLevel(ElevatorLevel.values()[currentLevel + 1]);
  }

  public void previousLevel() {
    if (currentLevel < 1) return;

    moveToLevel(ElevatorLevel.values()[currentLevel - 1]);
  }

  public void setCurrentLevelOffset() {
    if (currentLevel < 1) return;

    heightOffsets[currentLevel] = inputs.heightMeters - ElevatorLevel.values()[currentLevel].height;
  }

  public void runIntake(double speed) {
    io.applyIntakeDutyCycle(currentLevel == 1 ? speed * 0.75 : speed);
  }

  public void playMusic() {
    io.playMusic();
  }

  public void stopMusic() {
    io.stopMusic();
  }

  public static enum ElevatorLevel {
    FLOOR(0.3),
    L1(4),
    L2(6),
    L3(13.8),
    L4(23.85);

    public double height;

    ElevatorLevel(double height) {
      this.height = height;
    }
  }

  public static double[] heightOffsets = {
    0.0, 0.0, 0.0, 0.0
  };

  private ShuffleboardTab tab;

  private GenericEntry connectedEntry, velocityEntry, heightEntry, setpointEntry;

  @Override
  public void initTab() {
    tab = Shuffleboard.getTab("Elevator");

    connectedEntry = tab.add("Connected", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(0, 0)
      .withSize(2, 2)
      .getEntry();
    velocityEntry = tab.add("Velocity", inputs.velocityMetersPerSecond)
      .withWidget(BuiltInWidgets.kDial)
      .withPosition(0, 2)
      .withSize(2, 2)
      .withProperties(Map.of("min", -10.0, "max", 10.0))
      .getEntry();
    heightEntry = tab.add("Height", inputs.heightMeters)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(0, 4)
      .withSize(2, 1)
      .withProperties(Map.of("min", 0.0, "max", 30, "center", 15))
      .getEntry();
    setpointEntry = tab.add("Setpoint", inputs.heightSetpointMeters)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(0, 5)
      .withSize(2, 1)
      .withProperties(Map.of("min", 0.0, "max", 30, "center", 15))
      .getEntry();
  }

  @Override
  public void updateTab() {
    connectedEntry.setBoolean(inputs.elevatorConnected);
    velocityEntry.setDouble(inputs.velocityMetersPerSecond);
    heightEntry.setDouble(inputs.heightMeters);
    setpointEntry.setDouble(inputs.heightSetpointMeters);
  }
}
