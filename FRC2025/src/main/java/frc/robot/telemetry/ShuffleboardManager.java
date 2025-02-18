// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry;

import frc.robot.telemetry.annotations.AutoDashboardOutput;
import frc.robot.telemetry.annotations.AutoDashboardTunable;
import frc.robot.telemetry.annotations.DevelopmentTabLayout;
import frc.robot.telemetry.annotations.GraphDashboardOutput;
import frc.robot.util.StringUtils;

import static frc.robot.telemetry.reflect.AnnotationProcessor.*;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

/** Add your docs here. */
public class ShuffleboardManager {

    public class ShuffleboardDevTab {
        public ShuffleboardTab tab;

        public boolean useAutoLayout;
        public List<GenericEntry> entries = new ArrayList<>();

        public ShuffleboardDevTab(String title, DevelopmentTabLayout layout) {
            tab = Shuffleboard.getTab(title);
            useAutoLayout = layout.useAutomaticLayout();
        }

        public void addEntry(Object obj, Field field) {
            Object fieldValue;
            try {
                fieldValue = field.get(obj);
            } catch (IllegalArgumentException e) {
                System.out.println("Illegal argument when creating entry for "
                    + field.getName() + " in the dev tab " + tab.getTitle());
                e.printStackTrace();
                return;
            } catch (IllegalAccessException e) {
                System.out.println("Field for the entry " + field.getName()
                    + " in the dev tab " + tab.getTitle()
                    + " cannot be accessed.");
                e.printStackTrace();
                return;
            }
            tab.add(
                StringUtils.pascalCaseToRegularCase(field.getName()),
                fieldValue).withWidget(getWidgetFromField(field));
        }
    }
    private int numTabs = 0;

    private Map<String, ShuffleboardDevTab> tabs = new HashMap<String,ShuffleboardDevTab>();

    public void generateShuffleboardTabs() {
        List<Class<?>> tabClasses = new ArrayList<>();
        try {
            tabClasses = findClassesAnnotatedWith("frc.robot.subsystems", DevelopmentTabLayout.class);
        } catch (IOException e) {
            System.out.println("Failed IO operation while trying to load Shuffleboard tabs");
            e.printStackTrace();
            return;
        } catch (ClassNotFoundException e) {
            System.out.println("Class not found while attempting to load Shuffleboard tabs");
            e.printStackTrace();
            return;
        }
        if (tabClasses.size() == 0) return;

        numTabs = tabClasses.size();

        for(int i = 0; i < numTabs; i++) {
            buildTab((DevelopmentTabLayout) tabClasses.get(i).getAnnotation(DevelopmentTabLayout.class),
                tabClasses.get(i));
        }
    }

    public void buildTab(DevelopmentTabLayout annotation, Object obj) {
        String title = annotation.key().isBlank()
            ? StringUtils.pascalCaseToRegularCase(annotation.getClass().getSimpleName())
            : StringUtils.pascalCaseToRegularCase(annotation.key());
        var tab = new ShuffleboardDevTab(title, annotation);
        tabs.put(title, tab);
        
        Field[] fields = getAllDeclaredFieldsWithAnnotations(annotation.getClass(),
            AutoDashboardOutput.class,
            AutoDashboardTunable.class,
            GraphDashboardOutput.class);
        for(Field f: fields) {
            tab.addEntry(obj, f);
        }
    }

    public static boolean checkIfDevelopmentTab(Object object) {
        Class<?> clazz = object.getClass();
        if (clazz.isAnnotationPresent(DevelopmentTabLayout.class)) {
            return true;
        }
        return false;
    }

    public static WidgetType getWidgetFromField(Field field) {
        WidgetType widget = BuiltInWidgets.kTextView;
        if (field.getGenericType() == Boolean.class
          ||field.getGenericType() == boolean.class) {
            widget = BuiltInWidgets.kBooleanBox;
        }
        return widget;
    }
}
