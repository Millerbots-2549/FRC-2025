// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class StringUtils {
    public static String camelCaseToSnakeCase(String str) {
        return str.replaceAll("([a-z])([A-Z]+)", "$1_$2").toLowerCase();
    }

    public static String snakeCaseToCamelCase(String str) {
        str = str.substring(0, 1).toUpperCase() + str.substring(1);
        while (str.contains("_")) {
            str = str.replaceFirst("_[a-z]", String.valueOf(Character.toUpperCase(str.charAt(str.indexOf("_") + 1))));
        }
        return str;
    }

    public static String pascalCaseToRegularCase(String str) {
        return str.replaceAll("([a-z])([A-Z]+)", "$1 $2");
    }
}
