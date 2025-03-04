// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.vision.QuestNav;

/** Add your docs here. */
@SuppressWarnings("unused")
public class DriverDash implements DashboardPublisher {
    private DriveSubsystem driveSubsystem;
    private QuestNav questNav;

    private ShuffleboardTab tab;

    private GenericEntry autoStatusEntry, teleopStatusEntry, timeLeftEntry, odometryTypeEntry;
    private ComplexWidget fieldEntry;
    private ComplexWidget leftCamera, rightCamera, driverCamera;

    private Field2d field;

    private static final StructArraySubscriber<Pose2d> activePathSubscriber =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/PathPlanner/activePath", Pose2d.struct)
            .subscribe(new Pose2d[0]);

    public DriverDash(DriveSubsystem driveSubsystem, QuestNav questNav) {
        this.driveSubsystem = driveSubsystem;
        this.questNav = questNav;
    }

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

        try {
            leftCamera = tab.addCamera("Left Camera", "left_cam", "mjpg:http://10.25.49.2:1181/?action=stream")
                .withProperties(Map.of("showControls", false))
                .withPosition(7, 0)
                .withSize(3, 2);
        } catch (Exception e) {
            System.out.println("Couldn't add left camera to driver dashboard");
            e.printStackTrace();
        }
        try {
            rightCamera = tab.addCamera("Right Camera", "right_cam", "mjpg:http://photonvision.local:1183/?action=stream")
                .withProperties(Map.of("showControls", false))
                .withPosition(10, 0)
                .withSize(3, 2);
        } catch (Exception e) {
            System.out.println("Couldn't add right camera to driver dashboard");
            e.printStackTrace();
        }
        try {
            driverCamera = tab.addCamera("Front Camera", "front_cam", "mjpg:http://photonvision.local:1186/stream.mjpg")
                .withProperties(Map.of("showControls", false))
                .withPosition(7, 2)
                .withSize(6, 4);
        } catch (Exception e) {
            System.out.println("Couldn't add front camera to driver dashboard");
            e.printStackTrace();
        }
    }

    @Override
    public void updateTab() {
        autoStatusEntry.setBoolean(DriverStation.isAutonomousEnabled());
        teleopStatusEntry.setBoolean(DriverStation.isTeleopEnabled());
        timeLeftEntry.setDouble(
            DriverStation.isAutonomous()
                ? 105.0 * ((15.0 - DriverStation.getMatchTime()) / 15.0)
                : 105.0 - DriverStation.getMatchTime());
        
                /* 
        field.setRobotPose(driveSubsystem.getPose());
        Pose2d[] activePath = activePathSubscriber.get(new Pose2d[0]);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.stream(activePath).collect(Collectors.toList()),
            new TrajectoryConfig(6.0, 6.0));
        field.getObject("traj").setTrajectory(trajectory);*/
    }
    
}
