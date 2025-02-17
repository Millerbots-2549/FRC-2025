// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * This is an implementation of the {@link GyroIO} interface which uses
 * a simulated gyroscope.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 * @see GyroIO
 */
public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.calibrating = false;
        inputs.yaw = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));
    }

    @Override
    public void zeroGyro(Rotation2d rotation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroGyro'");
    }
}
