// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int toplElevatorMotorCanId = 7;
  public static final int leftElevatorMotorCanId = 9;
  public static final int rightElevatorMotorCanId = 8;
  public static final int lowerLimitSwitchId = 9;
  public static final int upperLimitSwitchId = 8;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;
  public static final double speed = 0.1;

  // Setpoints
  public static final double homeSetpoint = 0.0;
  public static final double coralL4Setpoint = 210.0;
  public static final double coralL3Setpoint = 100.0;
  public static final double coralL2Setpoint = 45.0;
  public static final double aglaeL2Setpoint = 80.0;
  public static final double aglaeL3Setpoint = 130.0;
  public static final double processorSetpoint = 0.0;
  public static final double bargeSetPoint = 240.0;
  public static final double closeToL4 = 190.0;

  // PID Constants
  public static final double kP = 0.07;
  public static final double kI = 0.05;
  public static final double kD = 0.05;
}
