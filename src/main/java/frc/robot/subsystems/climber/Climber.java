// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new IntakeIO. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    SmartDashboard.putBoolean("FullyIn", isFullyIn());
    SmartDashboard.putBoolean("IsFullyOut", isFullyOut());
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public Command runServo(double speed) {
    return run(() -> io.setSpeedServo(speed));
  }

  public Command runServoAndClimber(double percent, double position) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public boolean isAt0deg() {
    return inputs.isAt0deg;
  }

  public boolean isAt90deg() {
    return inputs.isAt90deg;
  }

  public boolean isFullyIn() {
    return inputs.isFullyIn;
  }

  public boolean isFullyOut() {
    return inputs.isFullyOut;
  }

  public boolean limitSwitch() {
    return inputs.limitSwitch;
  }

  public boolean isAtMin() {
    if (limitSwitch()) {
      io.resetEncoder();
      return true;
    } else {
      return false;
    }
  }
}
