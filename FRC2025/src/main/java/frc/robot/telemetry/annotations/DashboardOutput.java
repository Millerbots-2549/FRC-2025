// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** 
 * This annotation is used to specify the properties of a field or method that should be displayed
 * in the Shuffleboard tab. This annotation can be used on fields or a method that returns a value.
 * 
 * <p>Supported Types:
 * <ul>
 *  <li><b>double / Double:</b> generates a text output by default</li>
 *  <li><b>int / Integer:</b> generates a text output by default</li>
 *  <li><b>boolean / Boolean:</b> generates a boolean box by default</li>
 *  <li><b>String:</b> generates a text output by default</li>
 *  <li><b>PowerDistributionPanel:</b> generates a Power Distribution Panel widget</li>
 *  <li>Any types allowed by subinterfaces of this interface (DashboardOutput.NumberBar, DashboardOutput.Dial, etc.</li>
 * </ul>
 * 
 * <p>If the field or method is not a type that is supported by a widget, a runtime exception will be thrown.
 * 
 * <p>Basic usage example:
 * <pre>{@code
 * &#64;DevelopmentTabLayout(name = "Example Tab")
 * public class ExampleClass {
 *    &#64;DashboardOutput
 *    private double exampleField = 0.0;
 * 
 *    &#64;DashboardOutput(name = "Example Method")
 *    public double getExampleMethod() {
 *       return 0.0;
 *    }
 * }
 * }</pre>
 * <p>Both the field and the method will be displayed in the Shuffleboard tab. The field will be displayed
 * with the name "exampleField" and the method will be displayed with the name "Example Method". Both will
 * be dsiplayed in the tab defined by the class, in a text output widget.
 * 
 * <p>Using the subinterfaces of this interface, you can specify the type of widget to display the field or
 * method in. For example, to display a field in a number bar widget, you can use the following code:
 * <pre>{@code
 *    &#64;DashboardOutput.NumberBar
 *private double exampleField = 0.0;
 * }</pre>
 * 
 * <p>Using the subinterfaces of this interface, you can also specify the properties of the widget to display
 * the field or method in. For example, to display a field in a number bar widget with a minimum value of 0 and
 * a maximum value of 10, you can use the following code:
 * <pre>{@code
 *     &#64;DashboardOutput.NumberBar(min = 0.0, max = 10.0)
 *private double exampleField = 0.0;
 * }</pre>
 * 
 * <p>You can also specify a method to call to get the value to display in the widget. For example, to display
 * the value of a field that may be private, you can use the following code:
 * <pre>{@code
 *  &#64;DashboardOutput(getterMethodName = "getExampleField")
 *private double exampleField = 0.0;
 * 
 *public double getExampleField() {
 *   return exampleField;
 *}
 * }</pre>
 * 
 * <p><b>#######</b>
 * <p><b>-Layout-</b>
 * <p><b>#######</b>
 * <p>By default, the widget will be placed in the next available row and column in the tab. You can specify
 * the row and column to place the widget in by using the row and col properties. For example, to place a
 * widget in row 2 and column 3, you can use the following code:
 * <pre>{@code
 * &#64;DashboardOutput(row = 2, col = 3)
 * }</pre>
 * 
 * <p> if a parent class has a {@link DevelopmentTabLayout} annotation, and the field "layoutType" is set to "Auto"
 * the widget will be placed automatically, unless the row or col are specified. if the field "layoutType" is set
 * to "Grid" the widget will be placed in the next available row and column in the tab, unless the row or col are
 * specified.
 * 
 * <p> you can change the width and height of the widget by using the width and height properties. For example, to
 * make a widget 2 columns wide and 3 rows tall, you can use the following code:
 * <pre>{@code
 * &#64;DashboardOutput(width = 2, height = 3)
 * }</pre>
 * <p> This is not affected by the parent class.
 * 
 * <p>NOTE: If the width and height are not specified, the dimensions will be automaticalls set based on the widget
 * type.
 * 
 * @author <a href="https://github.com/linus-honer">Linus Honer</a>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface DashboardOutput {
    /**
     * The name of the widget to display in the Shuffleboard tab. If not specified, the name of the
     * field or method will be used.<br>
     * 
     * @return The name of the widget to display in the Shuffleboard tab.
     */
    public String name() default "";

    /**
     * The name of the tab to display the widget in. If not specified, the widget will be displayed
     * in the tab defined by it's class.<br>
     * 
     * @return The name of the tab to display the widget in.
     */
    public String tabName() default "";

    /**
     * The name of the method to call to get the value to display in the widget. If not specified,
     * the value of the field will be used.<br>
     * 
     * @return The name of the method to call to get the value to display in the widget.
     */
    public String getterMethodName() default "";

    /**
     * This is the row to place the widget in the Shuffleboard tab. If not specified, the widget will
     * be placed in the next available row.
     * <p> if the parent class has a {@link DevelopmentTabLayout}} annotation, and the field "layoutType"
     * is set to "Auto," the widget will be placed automatically, unless the row or col are specified.
     * @return The row to place the widget in the Shuffleboard tab.
     */
    public int row() default -1;

    /**
     * This is the column to place the widget in the Shuffleboard tab. If not specified, the widget will
     * be placed in the next available column.
     * <p> if the parent class has a {@link DevelopmentTabLayout}} annotation, and the field "layoutType"
     * is set to "Auto," the widget will be placed automatically, unless the row or col are specified.
     * @return The column to place the widget in the Shuffleboard tab.
     */
    public int col() default -1;

    /**
     * The width of the widget in the Shuffleboard tab. If not specified, the default width of the
     * widget will be used.
     * @return The width of the widget in the Shuffleboard tab.
     */
    public int width() default -1;

    /**
     * The height of the widget in the Shuffleboard tab. If not specified, the default height of the
     * widget will be used.
     * @return The height of the widget in the Shuffleboard tab.
     */
    public int height() default -1;

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
