// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int elevatorMotorLeaderCanId = 8;
  public static final int elevatorMotorFollowerCanId = 9;
  public static final int elevatorMotorFollower2CanId = 7;
  public static final int topLimitSwitchID = 8;
  public static final int bottomLimitSwitchID = 9;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;

  public static final double ELEVATOR_MAX_HEIGHT = 180.0;
  public static final double ELEVATOR_MIN_HEIGHT = 0.1;

  public static final double ELEVATOR_HOME_POSITION = 0.1;

  public static final double ELEVATOR_CORAL_PICKUP_POS = 0.1;
  public static final double ELEVATOR_ALGAE_PICKUP_FLOOR_POS = 0.1;

  public static final double ELEVATOR_ALGAE_SCORE_POS = 8;

  public static final double ELEVATOR_ALGAE_PICKUP_L2_POS = 0.1;
  public static final double ELEVATOR_ALGAE_PICKUP_L3_POS = 130.0;

  public static final double ELEVAOTR_SCORE_CORAL_L1_POS = 0.1;
  public static final double ELEVATOR_SCORE_CORAL_L2_POS = 50.0;
  public static final double ELEVATOR_SCORE_CORAL_L3_POS = 100.0;
  public static final double ELEVATOR_SCORE_CORAL_L4_POS = 180.0;
}
