// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controllers;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MathConstants;

/** Add your docs here. */
public class StateSpaceSystemController<S extends Num, I extends Num, O extends Num> implements AutoCloseable {
    private final String name;
    
    private final LinearSystemLoop<S, I, O> loop;

    private final Supplier<Vector<O>> outputSupplier;
    private final Consumer<Vector<I>> inputConsumer;

    private final double tolerance;

    private boolean atSetpoint = false;

    private final Notifier notifier;

    public StateSpaceSystemController(
            LinearSystemConfig<S,I,O> config,
            Supplier<Vector<O>> outputSupplier,
            Consumer<Vector<I>> inputConsumer,
            Vector<S> initialState) {
        
        this.name = config.getName();

        this.outputSupplier = outputSupplier;
        this.inputConsumer = inputConsumer;

        this.tolerance = config.getTolerance();

        this.loop = config.getLoop();
        loop.reset(initialState);

        notifier = new Notifier(this::periodic);
        notifier.startPeriodic(MathConstants.STATE_SPACE_DT);
    }

    private void periodic() {
        Vector<O> lastMeasurement = outputSupplier.get();
        loop.correct(lastMeasurement);

        loop.predict(MathConstants.STATE_SPACE_DT);

        Vector<I> inputs = new Vector<>(loop.getU());
        inputConsumer.accept(inputs);

        for(int i = 0; i < lastMeasurement.getNumRows(); i++) {
            double measurement = lastMeasurement.get(i);
            SmartDashboard.putNumber(name + " (" + i + ")", measurement);
        }

        atSetpoint = Math.abs(lastMeasurement.get(0) - getReference()) < tolerance;
    }

    public void setReference(Vector<S> reference) {
        loop.setNextR(reference);
    }

    public double getReference() {
        return loop.getNextR(0);
    }

    public boolean atSetpoint() {
        return atSetpoint;
    }

    @Override
    public void close() {
        notifier.close();
    }
}
