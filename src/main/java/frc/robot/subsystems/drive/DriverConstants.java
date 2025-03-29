// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class DriverConstants {
  public static double driveAngle = 0;
  public static int driveCANrangeId = 51;

  public static enum PoseState {
    DEFAULT,
    PROCESSOR,
    RIGHT_CORAL_STATION,
    LEFT_CORAL_STATION
  }

  public static enum ReefPoseStates {
    DEFAULT,
    BOTTOM_REEF,
    LEFT_BOTTOM_REEF,
    RIGHT_BOTTOM_REEF,
    LEFT_TOP_REEF,
    RIGHT_TOP_REEF,
    TOP_REEF
  }

  public static double MAGIC_INVALID_DEGREES_NUMBER = -333;
}
