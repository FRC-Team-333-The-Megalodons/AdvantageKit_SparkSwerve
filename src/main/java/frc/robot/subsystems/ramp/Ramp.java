// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Ramp extends SubsystemBase {

  private final RampIO io;
  private final RampIOInputsAutoLogged inputs = new RampIOInputsAutoLogged();

  /** Creates a new EndEffecterIO. */
  public Ramp(RampIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Ramp", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 6.0),
        () -> io.setVoltage(0.0));
  }

  public Command setRampPosition(double setpoint) {
    return runEnd(() -> io.setRampPosition(inputs.positionRad, setpoint), () -> io.setVoltage(0.0));
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
