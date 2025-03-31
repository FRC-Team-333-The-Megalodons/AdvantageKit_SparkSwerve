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
  public static final double coralL4Setpoint = 19.5;
  public static final double coralL3Setpoint = 8.8;
  public static final double coralL2Setpoint = 3;
  public static final double aglaeL2Setpoint = 6; // 25.0;
  public static final double aglaeL3Setpoint = 11.8;
  public static final double processorSetpoint = 0.0;
  public static final double netSetPoint = 23.0;
  public static final double closeToL4 = 19.0;

  // PID Constants (REV)
  public static final double kP_REV = 0.006;
  public static final double kI_REV = 0.0;
  public static final double kD_REV = 0.0;

  // PID Constants (CTRE)
  public static final double kP_CTRE = 0.12;
  public static final double kI_CTRE = 0.0;
  public static final double kD_CTRE = 0.02;
  public static final double kS_CTRE = 0.1;
  public static final double kV_CTRE = 0.12;
  public static final double kA_CTRE = 0.02;
  public static final double kG_CTRE = 0.16;
}
