// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.DriveConstants.*;

/** Add your docs here. */
public class PathfindingCommands {
    public static final PathConstraints SLOW_PATH_CONSTRAINTS = new PathConstraints(
        MAX_SPEED_METERS_PER_SECOND / 3,
        MAX_ACCELERATION / 3,
        MAX_ANGULAR_VELOCITY,
        MAX_ANGULAR_ACCELERATION);
    public static final PathConstraints FAST_PATH_CONSTRAINTS = new PathConstraints(
        MAX_SPEED_METERS_PER_SECOND / 1.5,
        MAX_ACCELERATION / 3,
        MAX_ANGULAR_VELOCITY,
        MAX_ANGULAR_ACCELERATION);

    public static Command pathfindToPoint(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, FAST_PATH_CONSTRAINTS);
    }

    public static Command pathfindToPointWhileAvoiding(
            Pose2d pose, Supplier<Pose2d> currentPoseSupplier,
            Supplier<List<Pair<Translation2d, Translation2d>>> obstacleSupplier,
            Notifier obstaclesChangedNotifier) {

        Command pathfindingCommand = pathfindToPoint(pose);
        obstaclesChangedNotifier.setCallback(
            () -> Pathfinding.setDynamicObstacles(
                obstacleSupplier.get(),
                currentPoseSupplier.get().getTranslation()));
        return pathfindingCommand;
    }

    public static PathPlannerPath autoGeneratePath(
            Pose2d startPose,
            Pose2d endPose,
            Translation2d currentLinearVelocity) {

        double velocityNorm = currentLinearVelocity.getNorm();
        IdealStartingState idealStartingState = new IdealStartingState(
            velocityNorm,
            startPose.getRotation());
        GoalEndState goalEndState = new GoalEndState(0.0, endPose.getRotation());

        Translation2d startTranslation = startPose.getTranslation();
        Translation2d endTranslation = endPose.getTranslation();
        Translation2d controlTranslation = startTranslation.plus(
            startTranslation.minus(endTranslation).div(2));

        double velocityProp = MAX_SPEED_METERS_PER_SECOND / velocityNorm;
        Translation2d idealControlTranslation = startTranslation.plus(currentLinearVelocity);
        controlTranslation = (controlTranslation.times(1 - velocityProp))
            .plus(idealControlTranslation.times(velocityProp));

        boolean reachableWithStraightLine = true;
        // TODO: calculate if the thingy can go in a straight line
        if(reachableWithStraightLine) {
            List<RotationTarget> rotationTargets = List.of(new RotationTarget(1, endPose.getRotation()));
            return new PathPlannerPath(
                List.of(
                    new Waypoint(null, startPose.getTranslation(), controlTranslation),
                    new Waypoint(controlTranslation, endPose.getTranslation(), null)
                ),
                rotationTargets,
                Collections.emptyList(),
                Collections.emptyList(),
                Collections.emptyList(),
                FAST_PATH_CONSTRAINTS,
                idealStartingState,
                goalEndState,
                false);
        } else {
            return null;
        }
    }
}
