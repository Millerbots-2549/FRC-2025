// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class SeperateControllerOI implements OI {
    private final CommandXboxController driverController;
    private final CommandXboxController manipulatorController;

    public SeperateControllerOI(int driverPort, int manipulatorPort) {
        this.driverController = new CommandXboxController(driverPort);
        this.manipulatorController = new CommandXboxController(manipulatorPort);
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getButton(CommandXboxController controller, Button button) {
        switch (button) {
            case A: return controller.a();
            case B: return controller.b();
            case X: return controller.x();
            case Y: return controller.y();
            case POV_UP: return controller.povUp();
            case POV_DOWN: return controller.povDown();
            case POV_LEFT: return controller.povLeft();
            case POV_RIGHT: return controller.povRight();
            case BACK: return controller.back();
            case START: return controller.start();
            case LEFT_STICK_BUTTON: return controller.leftStick();
            case RIGHT_STICK_BUTTON: return controller.rightStick();
            default: return controller.a();
        }
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getBumper(CommandXboxController controller, Bumper bumper) {
        if (bumper == Bumper.LB) {
            return controller.leftBumper();
        }
        return controller.rightBumper();
    }

    private edu.wpi.first.wpilibj2.command.button.Trigger getTrigger(CommandXboxController controller, Trigger trigger) {
        if (trigger == Trigger.LT) {
            return controller.leftTrigger();
        }
        return controller.rightTrigger();
    }

    @Override
    public void onDriveButtonPressed(Button button, Command command) {
        getButton(driverController, button).onTrue(command);
    }

    @Override
    public void whileDriveButtonPressed(Button button, Command command) {
        getButton(driverController, button).whileTrue(command);
    }

    @Override
    public void onDriveBumperPressed(Bumper bumper, Command command) {
        getBumper(driverController, bumper).onTrue(command);
    }

    @Override
    public void whileDriveBumperPressed(Bumper bumper, Command command) {
        getBumper(driverController, bumper).whileTrue(command);
    }

    @Override
    public void whileDriveTriggerPressed(Trigger trigger, Command command) {
        getTrigger(driverController, trigger).whileTrue(command);
    }

    @Override
    public void whileDriveTriggerPressedFullRange(Trigger trigger, Command command) {
        driverController.axisGreaterThan(trigger == Trigger.LT ? 2 : 3, 0.01).whileTrue(command);
    }

    @Override
    public boolean getDriveButtonDown(Button button) {
        return getButton(driverController, button).getAsBoolean();
    }

    @Override
    public boolean getDriveBumperDown(Bumper bumper) {
        return getBumper(driverController, bumper).getAsBoolean();
    }

    @Override
    public boolean getDriveTriggerDown(Trigger trigger) {
        return getTrigger(driverController, trigger).getAsBoolean();
    }

    @Override
    public double getDriveTriggerAxis(Trigger trigger) {
        return trigger == Trigger.LT
            ? driverController.getLeftTriggerAxis()
            : driverController.getRightTriggerAxis();
    }

    @Override
    public double getDriveLeftX() {
        return driverController.getLeftX();
    }

    @Override
    public double getDriveLeftY() {
        return driverController.getLeftY();
    }

    @Override
    public double getDriveRightX() {
        return driverController.getRightX();
    }

    @Override
    public double getDriveRightY() {
        return driverController.getRightY();
    }

    @Override
    public void setDriveRumble(RumbleType rumbleType, double rumble) {
        driverController.setRumble(rumbleType, rumble);
    }

    @Override
    public void onManipulatorButtonPressed(Button button, Command command) {
        getButton(manipulatorController, button).onTrue(command);
    }

    @Override
    public void whileManipulatorButtonPressed(Button button, Command command) {
        getButton(manipulatorController, button).whileTrue(command);
    }

    @Override
    public void onManipulatorBumperPressed(Bumper bumper, Command command) {
        getBumper(manipulatorController, bumper).onTrue(command);
    }

    @Override
    public void whileManipulatorBumperPressed(Bumper bumper, Command command) {
        getBumper(manipulatorController, bumper).whileTrue(command);
    }

    @Override
    public void whileManipulatorTriggerPressed(Trigger trigger, Command command) {
        getTrigger(manipulatorController, trigger).whileTrue(command);
    }

    @Override
    public void whileManipulatorTriggerPressedFullRange(Trigger trigger, Command command) {
        manipulatorController.axisGreaterThan(trigger == Trigger.LT ? 2 : 3, 0.01).whileTrue(command);
    }

    @Override
    public boolean getManipulatorButtonDown(Button button) {
        return getButton(manipulatorController, button).getAsBoolean();
    }

    @Override
    public boolean getManipulatorBumperDown(Bumper bumper) {
        return getBumper(manipulatorController, bumper).getAsBoolean();
    }

    @Override
    public boolean getManipulatorTriggerDown(Trigger trigger) {
        return getTrigger(manipulatorController, trigger).getAsBoolean();
    }

    @Override
    public double getManipulatorTriggerAxis(Trigger trigger) {
        return trigger == Trigger.LT
            ? manipulatorController.getLeftTriggerAxis()
            : manipulatorController.getRightTriggerAxis();
    }

    @Override
    public double getManipulatorLeftX() {
        return manipulatorController.getLeftX();
    }

    @Override
    public double getManipulatorLeftY() {
        return manipulatorController.getLeftY();
    }

    @Override
    public double getManipulatorRightX() {
        return manipulatorController.getRightX();
    }

    @Override
    public double getManipulatorRightY() {
        return manipulatorController.getRightY();
    }

    @Override
    public void setManipulatorRumble(RumbleType rumbleType, double rumble) {
        manipulatorController.setRumble(rumbleType, rumble);
    }
}
