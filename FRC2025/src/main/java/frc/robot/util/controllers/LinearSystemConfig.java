// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.Constants.MathConstants;

/** Add your docs here. */
public class LinearSystemConfig<S extends Num, I extends Num, O extends Num> {
    private String name;

    private LinearSystem<S,I,O> plant;
    private Vector<S> stateStdDev;
    private Vector<O> outputStdDev;

    private Vector<S> Q;
    private Vector<I> R;

    private Nat<S> states;
    private Nat<O> outputs;

    private double tolerance;

    public LinearSystemConfig(
            String name,
            LinearSystem<S,I,O> plant,
            Vector<S> stateStdDev,
            Vector<O> outputStdDev,
            Vector<S> Qelms,
            Vector<I> Relms,
            Nat<S> states,
            Nat<O> outputs,
            double tolerance) {

        this.name = name;

        this.plant = plant;
        this.stateStdDev = stateStdDev;
        this.outputStdDev = outputStdDev;

        this.Q = Qelms;
        this.R = Relms;

        this.states = states;
        this.outputs = outputs;

        this.tolerance = tolerance;
    }

    public KalmanFilter<S,I,O> getObserver() {
        return new KalmanFilter<S,I,O>(
            states,
            outputs,
            plant,
            stateStdDev,
            outputStdDev,
            MathConstants.STATE_SPACE_DT
        );
    }

    public LinearQuadraticRegulator<S,I,O> getController() {
        return new LinearQuadraticRegulator<S,I,O>(
            plant,
            Q,
            R,
            MathConstants.STATE_SPACE_DT
        );
    }

    public LinearSystemLoop<S,I,O> getLoop() {
        return new LinearSystemLoop<S,I,O>(
            plant,
            getController(),
            getObserver(),
            12.0,
            MathConstants.STATE_SPACE_DT
        );
    }

    public String getName() {
        return name;
    }

    public double getTolerance() {
        return tolerance;
    }

    public LinearSystem<S,I,O> getPlant() {
        return plant;
    }

    public Vector<S> getStateStdDev() {
        return stateStdDev;
    }

    public Vector<O> getOutputStdDev() {
        return outputStdDev;
    }

    public Vector<S> getQ() {
        return Q;
    }

    public Vector<I> getR() {
        return R;
    }

    public Nat<S> getStates() {
        return states;
    }

    public Nat<O> getOutputs() {
        return outputs;
    }
}
