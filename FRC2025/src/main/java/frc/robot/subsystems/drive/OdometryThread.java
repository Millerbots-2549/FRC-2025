// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class OdometryThread {
    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static OdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    public static OdometryThread getInstance() {
    if (instance == null) {
        instance = new OdometryThread();
    }
    return instance;
    }

    private OdometryThread() {
    notifier.setName("OdometryThread");
    }

    public void start() {
    if (timestampQueues.size() > 0) {
        notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY);
    }
    }

    /** Registers a Spark signal to be read from the thread. */
    public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    DriveSubsystem.odometryLock.lock();
    try {
        sparks.add(spark);
        sparkSignals.add(signal);
        sparkQueues.add(queue);
    } finally {
        DriveSubsystem.odometryLock.unlock();
    }
    return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    DriveSubsystem.odometryLock.lock();
    try {
        genericSignals.add(signal);
        genericQueues.add(queue);
    } finally {
        DriveSubsystem.odometryLock.unlock();
    }
    return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
        return queue;
        }

        private void run() {
        // Save new data to queues
        DriveSubsystem.odometryLock.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;

            // Read Spark values, mark invalid in case of error
            double[] sparkValues = new double[sparkSignals.size()];
            boolean isValid = true;
            for (int i = 0; i < sparkSignals.size(); i++) {
            sparkValues[i] = sparkSignals.get(i).getAsDouble();
            if (sparks.get(i).getLastError() != REVLibError.kOk) {
                isValid = false;
            }
            }

            // If valid, add values to queues
            if (isValid) {
            for (int i = 0; i < sparkSignals.size(); i++) {
                sparkQueues.get(i).offer(sparkValues[i]);
            }
            for (int i = 0; i < genericSignals.size(); i++) {
                genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
            }
            for (int i = 0; i < timestampQueues.size(); i++) {
                timestampQueues.get(i).offer(timestamp);
            }
            }
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
    }
}