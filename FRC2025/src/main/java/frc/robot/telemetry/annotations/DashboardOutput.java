// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Add your docs here. */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface DashboardOutput {
    public String name() default "";

    public String tabName() default "";

    public String getterMethodName() default "";

    public int row() default -1;

    public int col() default -1;

    public int width() default -1;

    public int height() default -1;

    BuiltInWidgets g = BuiltInWidgets.kTextView;

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface NumberBar {
        public double min() default 0.0;

        public double max() default 1.0;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Dial {
        public double min() default 0.0;

        public double max() default 1.0;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface VoltageView {
        public double min() default 0.0;

        public double max() default 1.0;

        public double center() default 0.0;

        public boolean isVertical() default false;

        public int numberOfTickMarks() default 10;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface PowerDistribution {
        public boolean showVoltageAndCurrent() default true;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Encoder {
        
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Accelerometer {
        public double min() default -1.0;
        
        public double max() default 1.0;

        public boolean showValue() default true;

        public int maxDigits() default 2;

        public boolean showTicks() default true;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface ThreeAxisAccelerometer {
        public double min() default -1.0;

        public double max() default 1.0;

        public boolean showValue() default true;

        public int maxDigits() default 2;

        public boolean showTicks() default true;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Gyro {
        public double tickSpacing() default 45;

        public double startingAngle() default 180;

        public boolean showTicks() default true;
    }
}
