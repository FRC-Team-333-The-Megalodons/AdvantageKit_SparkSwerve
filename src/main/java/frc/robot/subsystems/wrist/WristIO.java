// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionAbs = 0.0;
    public boolean atSetpoint = false;
    public boolean atL4Setpoint = false;
    public boolean atHomePosition = false;
    public boolean atAlgaeSetpoint = false;
    public boolean atL3Setpoint = false;
    public boolean atNetSetPoint = false;
    public boolean atNetLobSetPoint = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}

  public default void setWristPosition(double currentPos, double targetPos) {}
}
