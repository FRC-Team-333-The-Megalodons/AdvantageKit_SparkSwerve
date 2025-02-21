// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    // Logger.recordOutput("HopperEncoder", hopperEncoder.get());
    // SmartDashboard.putNumber("HopperEncoder", getPosition());
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 6.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  // public Command runHopper(double setPoint) {
  //   return run(() -> io.runHopperPIDController(getPosition(), setPoint));
  // }

  // public double getPosition() {
  //   return hopperEncoder.get();
  // }

  // public boolean rotationForHopper() {
  //   return (getPosition() >= 0);
  // }
}
