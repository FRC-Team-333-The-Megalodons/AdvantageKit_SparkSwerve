// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;

  public static boolean isPastSlowdownHeight = false;

  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // private final SysIdRoutine sysId;

  public Elevator(ElevatorIO io) {
    this.io = io;

    // sysId =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
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

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return run(() -> runCharacterization(0.0))
  //       .withTimeout(1.0)
  //       .andThen(sysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return run(() ->
  // runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    isPastSlowdownHeight = evaluateIsPastSlowdownHeight();
    SmartDashboard.putBoolean("ElevatorHigh", isPastSlowdownHeight);
  }

  public boolean evaluateIsPastSlowdownHeight() {
    if (upperLimit()) {
      return true;
    }

    if (atL4Setpoint()) {
      return true;
    }

    return inputs.position > ElevatorConstants.closeToL4;
  }

  public boolean lowerLimit() {
    if (inputs.lowerLimit) {
      io.resetEncoder();
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
