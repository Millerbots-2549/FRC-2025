// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class CharacterizationCommands {
    private static final double FF_START_DELAY = 2.0;
    private static final double FF_RAMP_RATE = 0.1;

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    public static Command feedforwardCharacterization(DriveSubsystem driveSubsystem) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            Commands.runOnce(
                () -> {
                velocitySamples.clear();
                voltageSamples.clear();
                }),

            Commands.run(
                    () -> {
                    driveSubsystem.runCharacterization(0.0);
                    },
                    driveSubsystem)
                .withTimeout(FF_START_DELAY),

            Commands.runOnce(timer::restart),

            Commands.run(
                    () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    driveSubsystem.runCharacterization(voltage);
                    velocitySamples.add(driveSubsystem.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                    },
                    driveSubsystem)

                .finallyDo(
                    () -> {
                    int n = velocitySamples.size();
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumXY = 0.0;
                    double sumX2 = 0.0;
                    for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                    }
                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("*** Drive FF Characterization Results ***");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }

    public static Command wheelRadiusCharacterization(DriveSubsystem driveSubsystem) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                    limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    driveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    driveSubsystem)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                    state.positions = driveSubsystem.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = driveSubsystem.getRotation();
                    state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                        var rotation = driveSubsystem.getRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                        double[] positions = driveSubsystem.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius =
                            (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                        NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println(
                            "********** Wheel Radius Characterization Results **********");
                        System.out.println(
                            "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println(
                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println(
                            "\tWheel Radius: "
                                + formatter.format(wheelRadius)
                                + " meters, "
                                + formatter.format(Units.metersToInches(wheelRadius))
                                + " inches");
                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
