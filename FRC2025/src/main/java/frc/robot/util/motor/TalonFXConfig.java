// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Add your docs here. */
public class TalonFXConfig {
    public String name = "untitled";
    public int canID;
    public TalonFXConfiguration config = new TalonFXConfiguration();

    public double rotorRatio = 1.0;
    public double MIN_POSITION = Double.NEGATIVE_INFINITY;
    public double MAX_POSITION = Double.POSITIVE_INFINITY;

    public double intertia = 0.5; // kg*m^2
}
