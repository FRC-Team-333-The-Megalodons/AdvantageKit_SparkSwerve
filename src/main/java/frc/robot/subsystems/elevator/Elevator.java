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
  private DigitalInput maxDownLimitSwitch =
      new DigitalInput(ElevatorConstants.maxDownLimitSwitchDigitalInputID);

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
    Logger.recordOutput("LowerLimitSwitch", lowerLimit());
    Logger.recordOutput("TopLimitSwitch", upperLimit());
  }

  // public boolean isTriggeredTopLimit() {
  //   return (maxUpLimitSwitch.get());
  // }

  // public boolean isTriggeredLowLimit() {
  //   return (maxDownLimitSwitch.get());
  // }

  // public boolean isOkToMoveElevatorUp() {
  //   // Add your logic here
  //   if (isTriggeredTopLimit()) {
  //     return false;
  //   } else if (isTriggeredLowLimit()) { // add encoder logic here too
  //     return true;
  //   }
  //   return false; // Placeholder return value
  // }

  // public boolean isOkToMoveElevatorDown() {
  //   // Add your logic here
  //   if (isTriggeredLowLimit()) {
  //     return false;
  //   } else if (isTriggeredTopLimit()) { // add encoder logic here too
  //     return true;
  //   }
  //   return false; // Placeholder return value
  // }

  // public void runElevator(double speed) {
  //   if (speed < 0) {
  //     if (!isOkToMoveElevatorUp()) {
  //       stopElevator();
  //       return;
  //     }
  //   } else if (speed > 0) {
  //     if (!isOkToMoveElevatorDown()) {
  //       stopElevator();
  //       return;
  //     }
  //   }
  //   io.setVoltage(4.0 * speed);
  // }

  // private void stopElevator() {
  //   io.setVoltage(0.0);
  // }

  public boolean lowerLimit() {
    return maxDownLimitSwitch.get() ? false : true;
  }

  public boolean upperLimit() {
    return maxUpLimitSwitch.get() ? false : true;
  }
}
