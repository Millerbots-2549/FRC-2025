// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import static frc.robot.Constants.DescorerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MathConstants;
import frc.robot.subsystems.descorer.DescorerSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DescoreHigh extends Command {
  private static final double RAISE_TIME = 0.7;
  private static final double RAISE_SPEED = 1.8;

  private final DescorerSubsystem descorerSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private Timer timer;

  private enum CommandState {
    LOWERING,
    RAISING,
    DESCORING
  }

  private CommandState currentState = CommandState.LOWERING;

  private boolean shouldFinish = false;

  /** Creates a new DescoreHigh. */
  public DescoreHigh(DescorerSubsystem descorerSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.descorerSubsystem = descorerSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    this.timer = new Timer();

    addRequirements(descorerSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = CommandState.LOWERING;
    shouldFinish = false;

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    descorerSubsystem.runRoller(1.0);
    SmartDashboard.putNumber("HUIJIJAJIJJOQOJKKOQ", Math.abs(descorerSubsystem.getCurrentWristPosition().getRadians()
      - DESCORER_ON_POSITION.getRadians()));
    SmartDashboard.putBoolean("Descoring", currentState == CommandState.DESCORING);
    switch (currentState) {
      case LOWERING:
        elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR);
        descorerSubsystem.applyWristSetpoint(DESCORER_ON_POSITION);
        if (Math.abs(MathUtil.inputModulus(descorerSubsystem.getCurrentWristPosition().getRadians(), 0, MathConstants.TAU)
            - MathUtil.inputModulus(DESCORER_ON_POSITION.getRadians(), 0, MathConstants.TAU)) < 0.1) {
          timer.reset();
          currentState = CommandState.RAISING;
        }
        break;
      
      case RAISING:
        descorerSubsystem.applyWristSetpointStrong(DESCORER_ON_POSITION.plus(Rotation2d.fromRadians(
          MathUtil.clamp(timer.get() * RAISE_SPEED, 0, 1.5))));
        elevatorSubsystem.setElevatorPosition(ElevatorLevel.FLOOR.height + (timer.get() * RAISE_SPEED * 3.0));
        if (timer.get() > RAISE_TIME) {
          timer.reset();
          currentState = CommandState.DESCORING;
        }
        break;
    
      default:
        descorerSubsystem.applyWristSetpoint(DESCORER_OFF_POSITION);
        if (Math.abs(descorerSubsystem.getCurrentWristPosition().getRadians()
            - DESCORER_OFF_POSITION.getRadians()) < 0.15) {
          elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR);
          shouldFinish = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.moveToLevel(ElevatorLevel.FLOOR);
    descorerSubsystem.applyWristSetpoint(DESCORER_OFF_POSITION);
    descorerSubsystem.runRoller(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldFinish;
  }
}
