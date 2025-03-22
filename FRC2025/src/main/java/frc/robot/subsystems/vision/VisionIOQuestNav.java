// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.vision.QuestNav;

/** Add your docs here. */
public class VisionIOQuestNav implements VisionIO {
    protected final QuestNav questNav;

    public VisionIOQuestNav(QuestNav questNav) {
        this.questNav = questNav;

        questNav.zeroPosition();
        questNav.zeroHeading();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = questNav.connected();

        inputs.latestTargetObservation = new TargetObservation(
            new Rotation2d(), new Rotation2d(),
            0.0, 0.0, new Pose3d(), -1);

        List<PoseObservation> poseObservations = new LinkedList<>();
        if (questNav.getPose() != null) {
            Pose2d QNPose = questNav.getPose();
            Transform3d fieldToCamera = new Transform3d(
                QNPose.getX(),
                QNPose.getY(),
                0,
                new Rotation3d(0, 0, QNPose.getRotation().getRadians()));
            Transform3d fieldToRobot = fieldToCamera;
            Pose3d robotPose = new Pose3d(
                fieldToRobot.getTranslation(), fieldToRobot.getRotation());
            
            poseObservations.add(new PoseObservation(
                questNav.timestamp(),
                robotPose,
                0,
                0,
                0,
                PoseObservationType.QUESTNAV));
        }

        inputs.poseObservations = 
            poseObservations.toArray(new PoseObservation[poseObservations.size()]);
        
        inputs.tagIds = new int[0];

        Logger.recordOutput("QuestNavOutputs/Connected", questNav.connected());
        Logger.recordOutput("QuestNavOutputs/Pose", questNav.getPose());
        Logger.recordOutput("QuestNavOutputs/BatteryPercent", questNav.getBatteryPercent());
        Logger.recordOutput("QuestNavOutputs/TImestamp", questNav.timestamp());

        SmartDashboard.putBoolean("Questnav connected", questNav.connected());
    }

    @Override
    public boolean useAprilTags() {
        return false;
    }
}
