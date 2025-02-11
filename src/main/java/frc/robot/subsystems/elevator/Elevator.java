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

  private DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
  private DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);

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
    Logger.recordOutput("LowerLimitSwitch", isAtLowerLimit());
    Logger.recordOutput("TopLimitSwitch", isAtUpperLimit());

  }

  public boolean isAtLowerLimit() {
    if (bottomLimitSwitch.get()) {
      return false;
    } else {
      io.resetEncoder();
      return true;
    }
  }

  public boolean isAtUpperLimit() {
    return topLimitSwitch.get() ? false : true;
  }

  private void stopElevator() {
    io.setVoltage(0.0);
  }

  public void resetEncoder() {
    if (bottomLimitSwitch.get() == true) {
      io.setValue(0);
    }
  }

  public Command setElevatorPosition(double setpoint) {
    return runEnd(() -> io.runElevatorPIDController(setpoint), () -> io.setVoltage(0));
  }

  public boolean isElevatorAtMaxHeightPos() {
    return io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;   // add values in the constants file
  }

  public boolean isElevatorAtMinHeightPos() {
    return io.getPosition() >= ElevatorConstants.ELEVATOR_MIN_HEIGHT;    // add values in the constants file
  }

  public boolean isElevatorAtCoralPickupPos() {
    return io.getPosition() >= ElevatorConstants.ELEVATOR_CORAL_PICKUP;   // add values in the constants file
  }

  public boolean isOkToMoveElevatorUp() {
    // Add your logic here
    if (isAtUpperLimit()) {
      return false;
    } else if (isElevatorAtMinHeightPos()) { // add encoder logic here too
      return true;
    }
    return false; // Placeholder return value
  }

  public boolean isOkToMoveElevatorDown() {
    // Add your logic here
    if (isAtUpperLimit()) {
      return false;
    } else if (isElevatorAtMaxHeightPos()) { // add encoder logic here too
      return true;
    }
    return false; // Placeholder return value
  }

  public void runElevator(double speed) {
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
    io.setVoltage(4.0 * speed);
  }

}
