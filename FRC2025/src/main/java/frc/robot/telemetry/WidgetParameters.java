// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry;

import java.lang.reflect.Method;
import java.util.Map;

import frc.robot.telemetry.annotations.DashboardOutput;

/** Add your docs here. */
public class WidgetParameters {
    public final String name;
    public final String tabName;

    public final String getterMethodName;

    public final int row;
    public final int col;
    public final int width;
    public final int height;

    public Map<String, Object> parameters;

    public WidgetParameters(DashboardOutput source) {
        this.name = source.name();
        this.tabName = source.tabName();
        this.row = source.row();
        this.col = source.col();
        this.width = source.width();
        this.height = source.height();
        this.getterMethodName = source.getterMethodName();
    }

    public Method getGetterMethod(Class<?> clazz) {
        try {
            return clazz.getMethod(getterMethodName);
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
            return null;
        }
    }

    public Object getParameter(String key) {
        return parameters.get(key);
    }

    public double getDoubleParameter(String key) {
        return (double) parameters.get(key);
    }

    public int getIntParameter(String key) {
        return (int) parameters.get(key);
    }

    public String getStringParameter(String key) {
        return (String) parameters.get(key);
    }

    public boolean getBooleanParameter(String key) {
        return (boolean) parameters.get(key);
    }
}
