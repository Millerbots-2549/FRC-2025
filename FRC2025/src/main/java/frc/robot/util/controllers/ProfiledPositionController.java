// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controllers;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ProfiledPositionController {
    private double dt = 0.02;
    private AsymmetricTrapezoidProfile.Constraints profileConstraints ;
    private AsymmetricTrapezoidProfile.State setPoint = new AsymmetricTrapezoidProfile.State();
    private AsymmetricTrapezoidProfile.State goal = new AsymmetricTrapezoidProfile.State();

    private boolean continuousInputEnabled = false;

    private AsymmetricTrapezoidProfile.State initialSetpoint =
        new AsymmetricTrapezoidProfile.State();
    private double feedforwardVolts = 0.0;
    private double minimumInput = 0.0;
    private double maximumInput = 0.0;

    public ProfiledPositionController(
            AsymmetricTrapezoidProfile.Constraints constraints,
            double period) {
        this.profileConstraints = constraints;
        this.dt = period;
    }

    public ProfiledPositionController(
            AsymmetricTrapezoidProfile.Constraints constraints) {
        this.profileConstraints = constraints;
    }

    public ProfiledPositionController(
            TrapezoidProfile.Constraints constraints,
            double period) {
        this.profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
        this.dt = period;
    }

    public ProfiledPositionController(
            TrapezoidProfile.Constraints constraints) {
        this.profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
    }

    public void setConstraints(AsymmetricTrapezoidProfile.Constraints constraints) {
        profileConstraints = constraints;
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        profileConstraints = new AsymmetricTrapezoidProfile.Constraints(constraints);
    }

    public AsymmetricTrapezoidProfile.State getGoal() {
        return goal;
    }

    public AsymmetricTrapezoidProfile.State getTarget() {
        return setPoint;
    }

    public AsymmetricTrapezoidProfile.State getInitialTarget() {
        return initialSetpoint;
    }

    public double getProfileDirection() {
        return Math.signum(goal.position - setPoint.position);
    }

    public void setReference(
            double reference,
            double measurement,
            Function<AsymmetricTrapezoidProfile.State, Double> feedforward) {
        if (reference != goal.position) {
            if (continuousInputEnabled) {
                double errorBound = (maximumInput - minimumInput) / 2.0;
                double goalMinDistance =
                    MathUtil.inputModulus(goal.position - measurement, -errorBound, errorBound);
                double setpointMinDistance =
                    MathUtil.inputModulus(setPoint.position - measurement, -errorBound, errorBound);

                goal.position = goalMinDistance + measurement;
                setPoint.position = setpointMinDistance + measurement;
            }
            goal = new AsymmetricTrapezoidProfile.State(reference, 0);
            initialSetpoint = setPoint;
        }

        var profile = new AsymmetricTrapezoidProfile(profileConstraints, goal, setPoint);
        setPoint = profile.calculate(dt);
        feedforwardVolts = feedforward.apply(setPoint);
    }

    public double getCalculatedPosition() {
        return setPoint.position;
    }

    public double getCalculatedFeedForward() {
        return feedforwardVolts;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        this.continuousInputEnabled = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public void disableContinuousInput() {
        this.continuousInputEnabled = false;
    }

    public boolean isContinuousInputEnabled() {
        return continuousInputEnabled;
    }

    public boolean atGoal() {
        return setPoint.equals(goal);
    }
}
