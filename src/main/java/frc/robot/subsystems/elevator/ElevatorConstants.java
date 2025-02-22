// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int toplEvatorMotorCanId = 7;
  public static final int leftElevatorMotorCanId = 9;
  public static final int rightElevatorMotorCanId = 8;
  public static final int lowerLimitSwitchId = 9;
  public static final int upperLimitSwitchId = 8;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;
  public static final double speed = 0.1;

  // Setpoints
  public static final double homeSetpoint = 0.0;
  public static final double coralL4Setpoint = 200.0;
  public static final double coralL3Setpoint = 95.0;
  public static final double coralL2Setpoint = 35.0;
  public static final double aglaeL2Setpoint = 80.0;
  public static final double aglaeL3Setpoint = 130.0;
  public static final double processorSetpoint = 0.0;
  public static final double bargeSetPoint = 200.0;
}
