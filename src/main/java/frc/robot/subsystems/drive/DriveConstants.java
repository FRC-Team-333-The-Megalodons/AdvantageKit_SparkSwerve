// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class DriveConstants {
  public static double desiredAngle = 0.0;

  public static class FieldConstants {
    static final double HALF_FILED_Y_MAG = 4;
    static final double MAX_Y_MAG = 8;
    static final double MIN_Y_MAG = 0;
    static final double MAX_CORAL_X_MAG = 3;
  }

  public static enum PoseStates {
    DEFAULT,
    LEFT_CORAL_STATION,
    RIGHT_CORAL_STATION,
    PROCESSOR
  }
}
