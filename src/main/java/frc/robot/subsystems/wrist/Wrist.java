package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(0);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    Logger.recordOutput("WristEncoder", wristEncoder.get());
    SmartDashboard.putNumber("WristEncoder", getPosition());
    Logger.recordOutput("AtTargetWrist", io.atTarget());
    SmartDashboard.putBoolean("AtTargetWrist", io.atTarget());
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 6.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public double getPosition() {
    return wristEncoder.get();
  }

  public boolean rotationForWrist() {
    return (getPosition() >= 0);
  }

  public Command setWristPosition(double setpoint) {
    return runEnd(() -> io.runWristPIDController(getPosition(), setpoint), () -> io.setVoltage(0));
  }

  public Command setWristPositionFeedForward(double setpoint) {
    return runEnd(() -> io.runWristPIDControllerFeedForward(setpoint), () -> io.setVoltage(0));
  }
}
