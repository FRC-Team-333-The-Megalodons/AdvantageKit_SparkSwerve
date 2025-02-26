// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;

  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public Command setElevatorPosition(double setpoint, boolean down) {
    return runEnd(() -> io.setElevator(inputs.position, setpoint, down), () -> io.setVoltage(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator Pos", io.getElevatorPosition());
  }

  public boolean lowerLimit() {
    io.resetEncoder();
    if (inputs.lowerLimit) {
      return true;
    } else {
      return false;
    }
  }

  public boolean upperLimit() {
    return inputs.upperLimit;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  public boolean atL4Setpoint() {
    return inputs.atL4Setpoint;
  }
}
