// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean isFullyOut = false;
    public boolean isFullyIn = false;
    public boolean limitSwitch = false;
    public boolean isAt90deg = false;
    public boolean isAt0deg = false;
    public double servoPosition = 0.0;
    public boolean climberAt90deg = false;
    public boolean climberAt0deg = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}

  public default void resetEncoder() {}

  public default void setServoPosition(double speed) {}

  public default void setClimberPos(double currentPos, double tragetPos) {}
}
