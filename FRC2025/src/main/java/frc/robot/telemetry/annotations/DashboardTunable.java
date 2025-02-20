// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Add your docs here. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface DashboardTunable {
    public String key() default "";

    public double min() default 0.0;

    public double max() default 1.0;
}
