// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is an implementation of the {@link GyroIO} interface which uses
 * a NavX gyro connected to the roborio via MXP SPI
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
public class GyroIONavX implements GyroIO {
    /** The NavX gyro */
    private final AHRS gyro;

    private Rotation2d offset;

    private int successive_error_count = 0;

    private int NUM_IGNORED_SUCCESSIVE_ERRORS = 50;
    private boolean TRACE = false;
    private boolean AHRS_USB_CONNECTED = false;
    SPI port = null;
    
    public GyroIONavX() {
        gyro = new AHRS(AHRS.NavXComType.kUSB1, NavXUpdateRate.k50Hz);
        gyro.enableLogging(true);
        gyro.zeroYaw();

        gyro.getRotation3d();

        offset = Rotation2d.kZero;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.calibrating = gyro.isCalibrating();
        inputs.yaw = Rotation2d.fromDegrees(-gyro.getAngle()).minus(offset);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-gyro.getRawGyroZ());
        inputs.rot3d = gyro.getRotation3d();
        inputs.accel = new Translation3d(
                gyro.getWorldLinearAccelX(),
                gyro.getWorldLinearAccelY(),
                gyro.getWorldLinearAccelZ());
        
        inputs.updateCount = gyro.getUpdateCount();
        inputs.byteCount = gyro.getByteCount();
    }

    @Override
    public void zeroGyro(Rotation2d rotation) {
        offset = rotation.plus(Rotation2d.fromDegrees(-gyro.getAngle()));
    }
}
