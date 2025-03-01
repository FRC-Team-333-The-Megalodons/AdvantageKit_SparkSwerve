// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public class ElevatorConstants {
  //  motors
  public static final int elevatorMotorTopCanId = 7;
  public static final int elevatorMotorLeftCanId = 9;
  public static final int elevatorMotorRightCanId = 8;
  // limit switch
  public static final int topLimitSwitchID = 8;
  public static final int bottomLimitSwitchID = 9;
  // current
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;
  public static final double speed = 0.7;
  // grains
  // pid whole
  public static final double kP = 0.06;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  // pid L4
  public static final double kP4 = 0.012;
  public static final double kI4 = 0.0;
  public static final double kD4 = 0.0005;

  // FF
  public static final double kS = 0.0;
  public static final double kG = 0.11;
  public static final double kV = 3.6;
  public static final double kA = 0.6;
  // setpoints
  public static final double ELEVATOR_MAX_HEIGHT = 200.0;
  public static final double ELEVATOR_MIN_HEIGHT = 0.0;

  public static final double ELEVATOR_HOME_POSITION = 0.0;

  public static final double ELEVATOR_CORAL_PICKUP_POS = 0.0;
  public static final double ELEVATOR_ALGAE_PICKUP_FLOOR_POS = 0.0;

  public static final double ELEVATOR_ALGAE_SCORE_POS = 0;

  public static final double ELEVATOR_ALGAE_SCORE_PROCESSOR_POS = 8;
  public static final double ELEVATOR_ALGAE_SCORE_NET_POS = 210.0;

  public static final double ELEVATOR_ALGAE_PICKUP_L2_POS = 80.0;
  public static final double ELEVATOR_ALGAE_PICKUP_L3_POS = 130.0;

  public static final double ELEVATOR_SCORE_CORAL_L1_POS = 0.0;
  public static final double ELEVATOR_SCORE_CORAL_L2_POS = 35.0;
  public static final double ELEVATOR_SCORE_CORAL_L3_POS = 95.0;
  public static final double ELEVATOR_SCORE_CORAL_L4_POS = 200.0;
}
