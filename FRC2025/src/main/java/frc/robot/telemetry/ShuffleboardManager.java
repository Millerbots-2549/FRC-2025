// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import frc.robot.telemetry.annotations.DashboardOutput;
import frc.robot.telemetry.annotations.DevelopmentTabLayout;

/** Add your docs here. */
public class ShuffleboardManager {
    private List<ShuffleboardTabManager> tabs;

    private Map<Class<? extends Annotation>, WidgetWrapperCreator> annotationProcessConsumers;

    @SuppressWarnings("unused")
    private void registerFields(Object obj, Class<?> clazz, Set<Field> alreadyRegistered) {
        Set<Field> fields = Set.of(clazz.getDeclaredFields());

        for(Field field : fields) {
            if(checkIfFieldIsNull(field, obj) || alreadyRegistered.contains(field)) {
                return;
            }
            field.setAccessible(true);
            alreadyRegistered.add(field);

            for(Class<? extends Annotation> type : annotationProcessConsumers.keySet()) {
                for(Annotation annotation : field.getAnnotationsByType(type)) {
                    WidgetWrapperCreator consumer = annotationProcessConsumers.get(annotation.annotationType());
                    if (consumer != null) {
                        consumer.generateFromField(() -> {
                            try {
                                return field.get(obj);
                            } catch (IllegalAccessException e) {
                                e.printStackTrace();
                                return null;
                            }
                        },
                        new WidgetParameters((DashboardOutput) annotation),
                        getShuffleboardTabManager((DashboardOutput) annotation, 
                            clazz.getAnnotation(DevelopmentTabLayout.class)));
                    }
                }
            }
        }
    }

    private static boolean checkIfFieldIsNull(Field field, Object obj) {
        field.setAccessible(true);
        boolean isNull = true;
        try {
            isNull = field.get(obj) == null;
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return isNull;
    }

    @FunctionalInterface
    private interface WidgetWrapperCreator {
        void generateFromField(
            Supplier<Object> valueSupplier,
            WidgetParameters widgetParameters,
            ShuffleboardTabManager tab
        );
    }

    private ShuffleboardTabManager getShuffleboardTabManager(DashboardOutput output, DevelopmentTabLayout layout) {
        String tabName = output.tabName();
        if (tabName == null || tabName.isEmpty()) {
            tabName = layout.key();
        }
        if (layout == null) {
            layout = getTabLayoutFromName(tabName);
        }
        for(ShuffleboardTabManager tab : tabs) {
            if(tab.getTabName().equals(tabName)) {
                return tab;
            }
        }
        ShuffleboardTabManager newTab = new ShuffleboardTabManager(tabName, layout);
        tabs.add(newTab);
        return newTab;
    }

    private DevelopmentTabLayout getTabLayoutFromName(String tabName) {
        for(ShuffleboardTabManager tab : tabs) {
            if(tab.getTabName().equals(tabName)) {
                return tab.layout;
            }
        }
        return tabs.get(0).layout;
    }
}
