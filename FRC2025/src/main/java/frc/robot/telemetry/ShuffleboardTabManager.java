// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.telemetry.annotations.DevelopmentTabLayout;

/** Add your docs here. */
public class ShuffleboardTabManager {
    public ShuffleboardTab tab;
    public DevelopmentTabLayout layout;
    public String getTabName() { return tab.getTitle(); }

    private Map<String, WidgetParameters> widgets;

    private final boolean doAutoLayout;

    public ShuffleboardTabManager(String tabName, DevelopmentTabLayout layout) {
        this.tab = Shuffleboard.getTab(tabName);
        this.layout = layout;
        this.doAutoLayout = layout.layoutType() == "Auto";
    }

    public void addWidget(WidgetParameters params, String widgetType) {
        tab.add(params.name, widgetType)
            .withPosition(params.col, params.row)
            .withSize(params.width, params.height);
        widgets.put(params.name, params);
    }

    public void generateTab() {
        if(doAutoLayout) {
            // TODO: Auto layout
        } else {
            // TODO: Grid layout
        }
    }
}
