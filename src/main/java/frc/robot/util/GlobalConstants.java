// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class GlobalConstants {
    public static  final String MANUAL_MODE_KEY = "MANUAL_MODE";
    public static boolean isManualMode() {
      return SmartDashboard.getBoolean(MANUAL_MODE_KEY, true);
    }
}
