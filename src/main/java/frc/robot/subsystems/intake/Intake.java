// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final CANrange canRange = new CANrange(IntakeConstants.canRangeId);
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);

  DigitalInput dSensor = new DigitalInput(0);
  // private IntakeIOSpark intakeIOSpark = new IntakeIOSpark();

  /** Creates a new IntakeIO. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // Logger.recordOutput("CANRange", isTriggered());
    // Logger.recordOutput("CANRangeDistance", getDistance());
    Logger.recordOutput("WristEncoder", wristEncoder.get());
    Logger.recordOutput("DSensor", getPositionForDSensor());
  }

  public Command runPercent(double percent) {
    if (!io.inRange()) {
      return run(() -> io.setVoltage(0.0));
    } else {
      return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
    }
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 6.0),
        () -> io.setVoltage(0.0));
  }

  public Command runIntake(double setPoint) {
    return run(() -> io.runWristPIDController(getPosition(), setPoint));
  }

  //

  // public boolean isTriggered() {
  //   return canRange.getIsDetected().getValue();
  // }

  // public double getDistance() {
  //   return canRange.getDistance().getValueAsDouble();
  // }

  public double getPosition() {
    return wristEncoder.get();
  }

  public boolean rotationForWrist() {
    return (getPosition() >= 0);
  }

  public boolean getPositionForDSensor() {
    return dSensor.get();
  }
}
