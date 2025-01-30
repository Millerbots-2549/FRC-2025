// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * This is an implementation of the {@link GyroIO} interface which uses
 * a NavX gyro connected to the roborio via MXP SPI
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class GyroIONavX implements GyroIO {
    /** The NavX gyro */
    public AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    // Queues used by the odometry thread to update the robot's rotation for pose estimation.
    public Queue<Double> yawPositionQueue;
    public Queue<Double> yawTimestampQueue;
    
    public GyroIONavX() {
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(gyro::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yaw = Rotation2d.fromDegrees(-gyro.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-gyro.getRawGyroZ());

        // Updating queues for odometry thread
        inputs.odometryTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYaw =
            yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
