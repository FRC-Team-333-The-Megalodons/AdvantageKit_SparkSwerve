// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {
  @AutoLog
  public static class RampIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean atSetpoint = false;
    // public boolean digitalInputSensor = false;
    // public boolean limitSwitch2 = false;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(RampIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}

  public default void setRampPosition(double currentPos, double targetPos) {}

  public default void resetEncoder() {}

  public default double getRampPosition() {
    return -1;
  }

  public default double getAngle() {
    return 0.0;
  }
  public default void runRampServo(double deegree) {}}
  
