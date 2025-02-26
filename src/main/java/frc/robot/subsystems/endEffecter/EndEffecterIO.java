// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffecter;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffecterIO {
  @AutoLog
  public static class EndEffecterIOInputs {
    public boolean isConnected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double canRangeDistance = 0.0;
    public boolean isTriggered = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(EndEffecterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}
}
