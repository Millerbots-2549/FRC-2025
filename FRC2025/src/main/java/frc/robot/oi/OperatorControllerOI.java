// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class OperatorControllerOI implements OI {
    private final CommandXboxController operatorController;

    public OperatorControllerOI(int port) {
        operatorController = new CommandXboxController(port);
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getButton(Button button) {
        switch (button) {
            case A: return operatorController.a();
            case B: return operatorController.b();
            case X: return operatorController.x();
            case Y: return operatorController.y();
            case POV_UP: return operatorController.povUp();
            case POV_DOWN: return operatorController.povDown();
            case POV_LEFT: return operatorController.povLeft();
            case POV_RIGHT: return operatorController.povRight();
            case BACK: return operatorController.back();
            case START: return operatorController.start();
            case LEFT_STICK_BUTTON: return operatorController.leftStick();
            case RIGHT_STICK_BUTTON: return operatorController.rightStick();
            default: return operatorController.a();
        }
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getBumper(Bumper bumper) {
        if (bumper == Bumper.LB) {
            return operatorController.leftBumper();
        }
        return operatorController.rightBumper();
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getTrigger(Trigger trigger) {
        if (trigger == Trigger.LT) {
            return operatorController.leftTrigger();
        }
        return operatorController.rightTrigger();
    }

    @Override
    public void onDriveButtonPressed(Button button, Command command) {
        getButton(button).onTrue(command);
    }

    @Override
    public void whileDriveButtonPressed(Button button, Command command) {
        getButton(button).whileTrue(command);
    }

    @Override
    public void onDriveBumperPressed(Bumper bumper, Command command) {
        getBumper(bumper).onTrue(command);
    }

    @Override
    public void whileDriveBumperPressed(Bumper bumper, Command command) {
        getBumper(bumper).whileTrue(command);
    }

    @Override
    public void whileDriveTriggerPressed(Trigger trigger, Command command) {
        getTrigger(trigger).whileTrue(command);
    }

    @Override
    public boolean getDriveButtonDown(Button button) {
        return getButton(button).getAsBoolean();
    }

    @Override
    public boolean getDriveBumperDown(Bumper bumper) {
        return getBumper(bumper).getAsBoolean();
    }

    @Override
    public boolean getDriveTriggerDown(Trigger trigger) {
        return getTrigger(trigger).getAsBoolean();
    }

    @Override
    public double getDriveTriggerAxis(Trigger trigger) {
        return trigger == Trigger.LT
            ? operatorController.getLeftTriggerAxis()
            : operatorController.getRightTriggerAxis();
    }

    @Override
    public double getDriveLeftX() {
        return operatorController.getLeftX();
    }

    @Override
    public double getDriveLeftY() {
        return operatorController.getLeftY();
    }

    @Override
    public double getDriveRightX() {
        return operatorController.getRightX();
    }

    @Override
    public double getDriveRightY() {
        return operatorController.getRightY();
    }

    @Override
    public void setDriveRumble(RumbleType rumbleType, double rumble) {
        operatorController.setRumble(rumbleType, rumble);
    }

    @Override
    public void onManipulatorButtonPressed(Button button, Command command) {
        onDriveButtonPressed(button, command);
    }

    @Override
    public void whileManipulatorButtonPressed(Button button, Command command) {
        whileDriveButtonPressed(button, command);
    }

    @Override
    public void onManipulatorBumperPressed(Bumper bumper, Command command) {
        onDriveBumperPressed(bumper, command);
    }

    @Override
    public void whileManipulatorBumperPressed(Bumper bumper, Command command) {
        whileDriveBumperPressed(bumper, command);
    }

    @Override
    public void whileManipulatorTriggerPressed(Trigger trigger, Command command) {
        whileDriveTriggerPressed(trigger, command);
    }

    @Override
    public boolean getManipulatorButtonDown(Button button) {
        return getDriveButtonDown(button);
    }

    @Override
    public boolean getManipulatorBumperDown(Bumper bumper) {
        return getDriveBumperDown(bumper);
    }

    @Override
    public boolean getManipulatorTriggerDown(Trigger trigger) {
        return getDriveTriggerDown(trigger);
    }

    @Override
    public double getManipulatorTriggerAxis(Trigger trigger) {
        return getDriveTriggerAxis(trigger);
    }

    @Override
    public double getManipulatorLeftX() {
        return getDriveLeftX();
    }

    @Override
    public double getManipulatorLeftY() {
        return getDriveLeftY();
    }

    @Override
    public double getManipulatorRightX() {
        return getDriveRightX();
    }

    @Override
    public double getManipulatorRightY() {
        return getDriveRightY();
    }

    @Override
    public void setManipulatorRumble(RumbleType rumbleType, double rumble) {
        setDriveRumble(rumbleType, rumble);
    }

}
