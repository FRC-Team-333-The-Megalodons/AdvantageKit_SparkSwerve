// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;

  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private DigitalInput maxUpLimitSwitch = new DigitalInput(ElevatorConstants.maxUpLimitSwitchID);
  private DigitalInput maxTopUpLimitSwitch = new DigitalInput(ElevatorConstants.maxTopUpLimitSwitchID);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("LowerLimitSwitch", isTriggeredLowLimit());
    Logger.recordOutput("TopLimitSwitch", isTriggeredTopLimit());
  }

  public boolean isTriggeredLowLimit() {
    return maxUpLimitSwitch.get();
  }
  public boolean isTriggeredTopLimit() {
    return maxTopUpLimitSwitch.get();
  }

  public boolean isOkToMoveElevatorUp() {
    // Add your logic here
    return true; // Placeholder return value
  }

  public boolean isOkToMoveElevatorDown() {
    // Add your logic here
    return true; // Placeholder return value
  }

  public void runElevator(double speed) {
    // Negative number means moving trolley out; positive number means moving trolley in.
    if (speed < 0) {
      if (!isOkToMoveElevatorUp()) {
        stopElevator();
        return;
      }
    } else if (speed > 0) {
      if (!isOkToMoveElevatorDown()) {
        stopElevator();
        return;
      }
    }
    ElevatorIO.set(speed);
  }

  private void stopElevator() {
    ElevatorIO.set(0);
  }
}
