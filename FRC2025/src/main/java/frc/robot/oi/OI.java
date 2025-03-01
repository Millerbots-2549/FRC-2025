// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public interface OI {
    public static enum Button {
        A,
        B,
        X,
        Y,
        POV_UP,
        POV_DOWN,
        POV_LEFT,
        POV_RIGHT,
        BACK,
        START,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON
    }

    public static enum Bumper {
        LB,
        RB
    }

    public static enum Trigger {
        LT,
        RT
    }

    // Drive

    public void onDriveButtonPressed(Button button, Command command);

    public void whileDriveButtonPressed(Button button, Command command);

    public void onDriveBumperPressed(Bumper bumper, Command command);

    public void whileDriveBumperPressed(Bumper bumper, Command command);

    public void whileDriveTriggerPressed(Trigger trigger, Command command);

    public void whileDriveTriggerPressedFullRange(Trigger trigger, Command command);

    public boolean getDriveButtonDown(Button button);

    public boolean getDriveBumperDown(Bumper bumper);

    public boolean getDriveTriggerDown(Trigger trigger);

    public double getDriveTriggerAxis(Trigger trigger);

    public double getDriveLeftX();

    public double getDriveLeftY();

    public double getDriveRightX();

    public double getDriveRightY();

    public void setDriveRumble(RumbleType rumbleType, double rumble);

    // Manip

    public void onManipulatorButtonPressed(Button button, Command command);

    public void whileManipulatorButtonPressed(Button button, Command command);

    public void onManipulatorBumperPressed(Bumper bumper, Command command);

    public void whileManipulatorBumperPressed(Bumper bumper, Command command);

    public void whileManipulatorTriggerPressed(Trigger trigger, Command command);

    public void whileManipulatorTriggerPressedFullRange(Trigger trigger, Command command);

    public boolean getManipulatorButtonDown(Button button);

    public boolean getManipulatorBumperDown(Bumper bumper);

    public boolean getManipulatorTriggerDown(Trigger trigger);

    public double getManipulatorTriggerAxis(Trigger trigger);

    public double getManipulatorLeftX();

    public double getManipulatorLeftY();

    public double getManipulatorRightX();

    public double getManipulatorRightY();

    public void setManipulatorRumble(RumbleType rumbleType, double rumble);
}
