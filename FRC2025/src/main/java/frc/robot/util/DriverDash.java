// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Add your docs here. */
public class DriverDash implements DashboardPublisher {
    private ShuffleboardTab tab;

    private GenericEntry autoStatusEntry, teleopStatusEntry, timeLeftEntry;
    @SuppressWarnings("unused")
    private ComplexWidget fieldEntry;
    private ComplexWidget leftCamera, rightCamera, driverCamera;

    private Field2d field;

    @Override
    public void initTab() {
        field = new Field2d();
        field.setRobotPose(new Pose2d());

        tab = Shuffleboard.getTab("Main");

        autoStatusEntry = tab.add("Auto Started?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 2)
            .getEntry();
        teleopStatusEntry = tab.add("Teleop Started?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 2)
            .withSize(2, 2)
            .getEntry();
        timeLeftEntry = tab.add("Time left", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(2, 0)
            .withSize(5, 2)
            .withProperties(Map.of("min", 0.0, "max", 105.0, "center", 105.0 / 2.0))
            .getEntry();

        fieldEntry = tab.add("Field View", field)
            .withWidget(BuiltInWidgets.kField)
            .withPosition(2, 2)
            .withSize(5, 4);

        /*leftCamera = tab.addCamera("Left Camera", "left_cam", "mjpg:http://10.25.49.2:1181/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(7, 0)
            .withSize(3, 2);*/
        /*rightCamera = tab.addCamera("Right Camera", "right_cam", "mjpg:http://photonvision.local:1183/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(10, 0)
            .withSize(3, 2);*/
        driverCamera = tab.addCamera("Front Camera", "front_cam", "mjpg:http://photonvision.local:1186/stream.mjpg")
            .withProperties(Map.of("showControls", false))
            .withPosition(7, 2)
            .withSize(6, 4);
    }

    @Override
    public void updateTab() {
        autoStatusEntry.setBoolean(DriverStation.isAutonomousEnabled());
        teleopStatusEntry.setBoolean(DriverStation.isTeleopEnabled());
        timeLeftEntry.setDouble(
            DriverStation.isAutonomous()
                ? 105.0 * ((15.0 - DriverStation.getMatchTime()) / 15.0)
                : 105.0 - DriverStation.getMatchTime());
    }
    
}
