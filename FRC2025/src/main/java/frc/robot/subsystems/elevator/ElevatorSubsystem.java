// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public static enum ElevatorStage {
    GROUND(0),
    L1(0.5 - 0.35),
    L2(0.85 - 0.35 - 0.13),
    L3(1.25 - 0.35 - 0.13),
    L4(1.87 - 0.35);

    private final double heightMeters;

    private ElevatorStage(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public double getHeight() {
      return heightMeters;
    }

    public static ElevatorStage ofIndex(int index) {
      if (index == 0) return GROUND;
      else if (index == 1) return L1;
      else if (index == 2) return L2;
      else if (index == 3) return L3;
      else if (index == 4) return L4;
      else if (index < 0) return GROUND;
      else return L4;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Stage", getCurrentStage());
  }

  public void setHeight(double height) {
    io.setHeight(height);
  }

  public void setStage(ElevatorStage stage) {
    setHeight(stage.getHeight());
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public double getHeightMeters() {
    return inputs.elevatorHeightMeters;
  }

  public double getHeightFeet() {
    return Units.metersToFeet(getHeightMeters());
  }

  public double getHeightInches() {
    return Units.metersToInches(getHeightMeters());
  }

  public double getVelocityMetersPerSecond() {
    return inputs.elevatorVelocityMetersPerSecond;
  }

  public double getVelocityFeetPerSecond() {
    return Units.metersToFeet(getVelocityMetersPerSecond());
  }

  public double getVelocityInchesPerSecond() {
    return Units.metersToInches(getVelocityMetersPerSecond());
  }

  public ElevatorStage getCurrentStage() {
    ElevatorStage closest = ElevatorStage.GROUND;
    double currentClosest = Double.POSITIVE_INFINITY;
    for (ElevatorStage stage : ElevatorStage.values()) {
      if (Math.abs(getHeightMeters() - stage.getHeight()) < currentClosest) {
        closest = stage;
        currentClosest = Math.abs(getHeightMeters() - stage.getHeight());
      }
    }
    return closest;
  }

  public double getErrorFromStage(ElevatorStage stage) {
    return getHeightMeters() - stage.getHeight();
  }

  public double getErrorFromCurrentStage() {
    return getErrorFromStage(getCurrentStage());
  }

  public boolean isAtStage(ElevatorStage stage) {
    return Math.abs(getErrorFromStage(stage)) < 0.1;
  }

  public boolean isAtTop() {
    return getHeightMeters() >= ElevatorStage.L4.getHeight();
  }

  public boolean isAtBottom() {
    return getHeightMeters() <= ElevatorStage.GROUND.getHeight();
  }

  public Command raiseElevator() {
    return new InstantCommand(
      () -> setStage(ElevatorStage.ofIndex(getCurrentStage().ordinal() + 1)),
      this
    );
  }
  public Command lowerElevator() {
    return new InstantCommand(
      () -> setStage(ElevatorStage.ofIndex(getCurrentStage().ordinal() - 1)),
      this
    );
  }
}
