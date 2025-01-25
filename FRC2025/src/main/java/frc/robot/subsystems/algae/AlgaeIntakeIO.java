// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
/** Add your docs here. */
public interface AlgaeIntakeIO {
    @AutoLog
    public static class AlgaeIntakeIOInputs {
        public boolean rollerConnected = false;
        public boolean angleConnected = false;

        public double rollerVelocity = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrent = 0.0;

        public double angleVelocity = 0.0;
        public double angleAppliedVolts = 0.0;
        public Rotation2d anglePosition = new Rotation2d();
        public double angleCurrent = 0.0;

        public double currentAngle = 0.0;
    }

    public default void updateInputs(AlgaeIntakeIOInputs inputs) {};

    public default void setRollerVelocity(double velocity) {};

    public default void setRollerOpenLoop(double volts) {};

    public default void setAnglePosition(Rotation2d position) {};

    public default void setAngleOpenLoop(double volts) {};

    public default AlgaeIntakeGains getGains() { return new AlgaeIntakeGains(0, 0, 0, 0, 0, 0); };

    public default void setGains(AlgaeIntakeGains gains) {};

    public record AlgaeIntakeGains(
        double rollerKP, double rollerKI, double rollerKD, double rollerKS, double rollerKV, double rollerKA) {}
}
