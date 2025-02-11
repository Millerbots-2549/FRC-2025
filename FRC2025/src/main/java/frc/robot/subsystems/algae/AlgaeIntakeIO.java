// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface used to create different hardware implementstions of the
 * algae intake. (This includes the roller motor and the motor that turns
 * the intake up and down.)
 * 
 * <p>The inner class {@link AlgaeIntakeIOInputs} is used to store all of the
 * inputs that will be logged to <a href="https://docs.advantagescope.com">
 * AdvantageScope</a> (Included in WPILib as of 2025, located in the WPILib
 * Tools folder)</p>
 * <ul>
 *  <li>The method {@link updateInputs} is used to update a given {@link
 *     AlgaeIntakeIOInputs} object with the inputs calculated with this
 *     class. This should be changed for every implementation of this
 *     interface.</li>
 * </ul>
 * 
 * <p> This class also contains a record used for storing the PID values
 * for both motors in the algae intake. ({@link AlgaeIntakeGains})
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
*/
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

    /**
     * This method is used to update a given {@link AlgaeIntakeIOInputs} object with the 
     * inputs calculated with this class. This should be changed for every implementation
     * of this interface.
     * @param inputs An {@link AlgaeIntakeIOInputs} object that you would like to be updated
     *               With the values from this IO.
     */
    public default void updateInputs(AlgaeIntakeIOInputs inputs) {};

    /**
     * Runs the roller motor to run at a specific velocity in <b>rotations per second</b>
     * @param velocity The velocity in <b>rotations per second</b>
     */
    public default void setRollerVelocity(double velocity) {};

    /**
     * Runs the roller motor to run at a specific voltage
     * @param volts The voltage to run the motor at
     */
    public default void setRollerOpenLoop(double volts) {};

    /**
     * Runs the angle motor to run at a specific velocity in <b>rotations per second</b>
     * @param velocity The velocity in <b>rotations per second</b>
     */
    public default void setAnglePosition(Rotation2d position) {};
    public default void setAnglePosition(Rotation2d position, boolean resist) {};

    /**
     * Runs the angle motor to run at a specific voltage
     * @param volts The voltage to run the motor at
     */
    public default void setAngleOpenLoop(double volts) {};

    public default AlgaeIntakeGains getGains() { return new AlgaeIntakeGains(0, 0, 0, 0, 0, 0, 0, 0, 0); };

    public default void setGains(AlgaeIntakeGains gains) {};

    /**
     * A record containing PID & SVA values for the roller motor as well as PID values for the angle motor.
     */
    public record AlgaeIntakeGains(
        double rollerKP, double rollerKI, double rollerKD, double rollerKS, double rollerKV, double rollerKA,
        double angleKP, double angleKI, double angleKD) {}
}
