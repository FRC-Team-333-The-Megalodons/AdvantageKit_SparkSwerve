package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 6.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public Command setWristPosition(double setpoint) {
    return runEnd(
        () -> io.setWristPosition(inputs.positionAbs, setpoint), () -> io.setVoltage(0.0));
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  public boolean atL4Setpoint() {
    return inputs.atL4Setpoint;
  }

  public boolean atHomePosition() {
    return inputs.atHomePosition;
  }
  public boolean atAlgaeSetpoint(){
    return inputs.atAlgaeSetpoint;
  }
}
