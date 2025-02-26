// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

/** Add your docs here. */
public class RampConstants {
  public static final int rampCanId = 6;
  public static final int rampLimitSwitchId = -1;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 30;
  public static final double speed = 0.1;

  // PID Constants
  public static final double kP = 0.05;
  public static final double kI = 0.05;
  public static final double kD = 0.05;
}
