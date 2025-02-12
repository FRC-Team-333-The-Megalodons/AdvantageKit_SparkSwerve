// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffecter;

import static frc.robot.subsystems.endEffecter.EndEffecterConstants.*;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDStrip;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffecter extends SubsystemBase {

  private final EndEffecterIO io;
  private final EndEffecterIOInputsAutoLogged inputs = new EndEffecterIOInputsAutoLogged();
  private final CANrange canRange = new CANrange(canRangeId);

  /** Creates a new EndEffecterIO. */
  public EndEffecter(EndEffecterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("EndEffecter", inputs);
    Logger.recordOutput("CANRange", isTriggered());
    Logger.recordOutput("CANRangeDistance", getDistance());
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 6.0),
        () -> io.setVoltage(0.0));
  }

  public boolean isTriggered() {
    LEDStrip.setLEDs(Color.kGreen);
    return canRange.getIsDetected().getValue();
  }

  public double getDistance() {
    return canRange.getDistance().getValueAsDouble();
  }
}
