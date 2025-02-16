// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.logging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private int currentLevel = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SmartDashboard.putNumber("Elevator Height", inputs.heightMeters);
    SmartDashboard.putNumber("Elevator Setpoint", inputs.heightSetpointMeters);
    SmartDashboard.putNumber("Elevator Velocity", inputs.velocityMetersPerSecond);
    SmartDashboard.putBoolean("Elevator Connected", inputs.elevatorConnected);
    SmartDashboard.putBoolean("Elevator Out of Bounds", inputs.outOfBounds);
  }

  public void setElevatorPosition(double position) {
    io.applySetpointMeters(position);
  }

  public void setElevatorVelocity(double velocity) {
    //io.applyVelocityMetersPerSecond(velocity);
    io.applyDutyCycle(velocity);
  }

  public void moveToLevel(ElevatorLevel level) {
    setElevatorPosition(level.height);
    currentLevel = level.ordinal();
  }

  public void nextLevel() {
    if (currentLevel >= 4) return;

    moveToLevel(ElevatorLevel.values()[currentLevel + 1]);
  }

  public void previousLevel() {
    if (currentLevel <= 1) return;

    moveToLevel(ElevatorLevel.values()[currentLevel - 1]);
  }

  public void playMusic() {
    io.playMusic();
  }

  public void stopMusic() {
    io.stopMusic();
  }

  public static enum ElevatorLevel {
    FLOOR(0.5),
    L1(0.5),
    L2(7.5),
    L3(15.0),
    L4(24.0);

    public final double height;
    ElevatorLevel(double height) {
      this.height = height;
    }
  }
}
