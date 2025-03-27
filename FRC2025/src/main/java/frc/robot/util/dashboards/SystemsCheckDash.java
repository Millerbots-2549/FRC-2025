// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.dashboards;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.descorer.DescorerSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.DashboardPublisher;

/** Add your docs here. */
@SuppressWarnings("unused")
public class SystemsCheckDash implements DashboardPublisher {

    private final static String TAB_NAME = "System";

    private final DriveSubsystem driveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final DescorerSubsystem descorerSubsystem;
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    private GenericEntry elevatorLeftStatus, elevatorRightStatus;
    private GenericEntry descorerArmStatus, descorerWheelStatus;
    private GenericEntry algaeIntakeArmStatus, algaeIntakeRollerStatus;
    private GenericEntry frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private GenericEntry rioMemoryUsage;
    private GenericEntry rioCPUUsage;

    private ComplexWidget rightCam, leftCam;

    public SystemsCheckDash(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, DescorerSubsystem descorerSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.descorerSubsystem = descorerSubsystem;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    }

    @Override
    public void initTab() {
        elevatorLeftStatus = Shuffleboard.getTab(TAB_NAME).add("Left Elevator", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(3, 2)
            .getEntry();
        elevatorRightStatus = Shuffleboard.getTab(TAB_NAME).add("Right Elevator", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 0)
            .withSize(3, 2)
            .getEntry();
        descorerArmStatus = Shuffleboard.getTab(TAB_NAME).add("Descorer Arm", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 2)
            .withSize(3, 2)
            .getEntry();
        descorerWheelStatus = Shuffleboard.getTab(TAB_NAME).add("Descorer Wheel", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 2)
            .withSize(3, 2)
            .getEntry();
        algaeIntakeArmStatus = Shuffleboard.getTab(TAB_NAME).add("Algae Arm", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 4)
            .withSize(3, 2)
            .getEntry();
        algaeIntakeRollerStatus = Shuffleboard.getTab(TAB_NAME).add("Algae Roller", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 4)
            .withSize(3, 2)
            .getEntry();
        
        frontLeftModule = Shuffleboard.getTab(TAB_NAME).add("Front Left", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 0)
            .withSize(2, 2)
            .getEntry();
        frontRightModule = Shuffleboard.getTab(TAB_NAME).add("Front Right", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(8, 0)
            .withSize(2, 2)
            .getEntry();
        backLeftModule = Shuffleboard.getTab(TAB_NAME).add("Back Left", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 2)
            .withSize(2, 2)
            .getEntry();
        backRightModule = Shuffleboard.getTab(TAB_NAME).add("Back Right", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(8, 2)
            .withSize(2, 2)
            .getEntry();
        
        rioMemoryUsage = Shuffleboard.getTab(TAB_NAME).add("RIO Memory Usage", "0b/0b")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 5)
            .withSize(3, 1)
            .getEntry();
        rioCPUUsage = Shuffleboard.getTab(TAB_NAME).add("RIO CPU Usage", "0%")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(9, 5)
            .withSize(3, 1)
            .getEntry();
    }

    @Override
    public void updateTab() {
        if(DriverStation.isDisabled()) {
            elevatorLeftStatus.setBoolean(elevatorSubsystem.getInputs().elevatorConnected);
            elevatorRightStatus.setBoolean(elevatorSubsystem.getInputs().elevatorConnected);

            descorerArmStatus.setBoolean(descorerSubsystem.getInputs().wristConnected);
            descorerWheelStatus.setBoolean(descorerSubsystem.getInputs().rollerConnected);

            algaeIntakeArmStatus.setBoolean(algaeIntakeSubsystem.getInputs().angleConnected);
            algaeIntakeRollerStatus.setBoolean(algaeIntakeSubsystem.getInputs().rollerConnected);

            frontLeftModule.setBoolean(driveSubsystem.getModuleInputs(0).driveConnected
                && driveSubsystem.getModuleInputs(0).turnConnected
                && driveSubsystem.getModuleInputs(0).canCoderConnected);
            frontRightModule.setBoolean(driveSubsystem.getModuleInputs(1).driveConnected
                && driveSubsystem.getModuleInputs(1).turnConnected
                && driveSubsystem.getModuleInputs(1).canCoderConnected);
            backLeftModule.setBoolean(driveSubsystem.getModuleInputs(2).driveConnected
                && driveSubsystem.getModuleInputs(2).turnConnected
                && driveSubsystem.getModuleInputs(2).canCoderConnected);
            backRightModule.setBoolean(driveSubsystem.getModuleInputs(3).driveConnected
                && driveSubsystem.getModuleInputs(3).turnConnected
                && driveSubsystem.getModuleInputs(3).canCoderConnected);
        }
        
        rioMemoryUsage.setString(Runtime.getRuntime().totalMemory()
            - Runtime.getRuntime().freeMemory() + "b/"
            + Runtime.getRuntime().totalMemory() + "b "
            + (100 * ((double)Runtime.getRuntime().freeMemory() / (double)Runtime.getRuntime().totalMemory())) + "%");
    }
    
}
