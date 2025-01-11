// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIONavX implements GyroIO {
    public AHRS gyro;
    public Queue<Rotation2d> yawPositionQueue;
    public Queue<Double> yawTimestampQueue;
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }
}
