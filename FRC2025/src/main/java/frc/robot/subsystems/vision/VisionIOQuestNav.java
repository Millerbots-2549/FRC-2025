// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.vision.QuestNav;

/** Add your docs here. */
public class VisionIOQuestNav implements VisionIO {
    protected final QuestNav questNav;
    protected final Transform3d robotToCamera;

    public VisionIOQuestNav(QuestNav questNav, Transform3d robotToCamera) {
        this.questNav = questNav;
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = questNav.connected();

        inputs.latestTargetObservation = null;

        List<PoseObservation> poseObservations = new LinkedList<>();
        if (questNav.getPose() != null) {
            Pose2d QNPose = questNav.getPose();
            Quaternion QNQuaternion = questNav.getQuaternion();
            Transform3d fieldToCamera = new Transform3d(
                QNPose.getX(),
                QNPose.getY(),
                0,
                new Rotation3d(QNQuaternion));
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
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
    }

    @Override
    public boolean useAprilTags() {
        return false;
    }
}
