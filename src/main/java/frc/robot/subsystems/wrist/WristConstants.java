// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

public class WristConstants {
  public static final int wristCanId = 4;
  public static final int wristEncoderId = 0;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 30;
  public static final double speed = 0.1;

  // Wrist Setpoints
  public static final double homeSetpoint = 0.52;
  public static final double coralL23Setpoint = 0.49;
  public static final double coralL4Setpoint = 0.37;
  public static final double processorSetpoint = 0.09;
  public static final double netSetPoint = 0.4;
  public static final double aglaeSetpoint = 0.12;
  public static final double algaeHomeSetpoint = 0.244;

  // PID Constants
  public static final double kP = 1.5;
  public static final double kI = 0.05;
  public static final double kD = 0.05;

  // OnBoard PID Controller
  public static final double kP_CTRE = 0;
  public static final double kI_CTRE = 0;
  public static final double kD_CTRE = 0;
}
