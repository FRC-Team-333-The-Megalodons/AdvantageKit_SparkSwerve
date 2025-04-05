// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.ramp.Ramp;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;

  public static double SERVO_UNLOCKED = 0;
  public static double SERVO_LOCKED = 1;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new IntakeIO. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    SmartDashboard.putBoolean("FullyIn", isFullyIn());
    SmartDashboard.putBoolean("IsFullyOut", isFullyOut());
    Logger.recordOutput("Climber/RemainingTimeForRatchet", remainingTimeForRatchet());
  }

  public void resetClimberMotorSensor() {}

  public Command runPercent(double percent, double servoPosition) {
    return runEnd(
        () -> // Command execute
        {
          if (servoPosition == Climber.SERVO_LOCKED) {
            resetTimer();
          } else if (servoPosition == Climber.SERVO_UNLOCKED) {
            startTimer();
          }
          io.setServoPosition(servoPosition);
          io.setVoltage(percent * 12.0);
        },
        () -> // On Command End
        {
          if (servoPosition == Climber.SERVO_UNLOCKED) {
            stopTimer();
          }
          io.setVoltage(0.0);
          // Don't fry the servo!
          io.stopServo();
        });
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  long collectiveServoOutTime = 0;
  long lastServoOutStart = -1;

  public void resetTimer() {
    collectiveServoOutTime = 0;
    lastServoOutStart = -1;
  }

  public void startTimer() {
    lastServoOutStart = System.currentTimeMillis();
  }

  public void stopTimer() {
    if (lastServoOutStart <= 0) {
      return;
    }
    double elapsed = System.currentTimeMillis() - lastServoOutStart;
    collectiveServoOutTime += elapsed;
  }

  public double remainingTimeForRatchet() {

    return ClimberConstants.TIME_FOR_SERVO_TO_UNLOCK;
    /*
    if (collectiveServoOutTime >= ClimberConstants.TIME_FOR_SERVO_TO_UNLOCK) {
      return 0;
    }

    return ClimberConstants.TIME_FOR_SERVO_TO_UNLOCK - collectiveServoOutTime;
    */
  }

  public Command runServoToPosition(double position) {
    return runEnd(
        () -> {
          if (position == Climber.SERVO_LOCKED) {
            resetTimer();
          } else if (position == Climber.SERVO_UNLOCKED) {
            startTimer();
          }
          io.setServoPosition(position);
        },
        () -> {
          if (position == Climber.SERVO_UNLOCKED) {
            stopTimer();
          }
          // Stop stressing the servo here
          io.stopServo();
        });
  }

  public Command setClimberPosition(double setPoint) {
    return runEnd(() -> io.setClimberPos(inputs.positionRad, setPoint), () -> io.setVoltage(0.0));
  }

  public boolean isAt0deg() {
    return inputs.isAt0deg;
  }

  public boolean isAt90deg() {
    return inputs.isAt90deg;
  }

  public boolean isFullyIn() {
    return inputs.isFullyIn;
  }

  public boolean isFullyOut() {
    return inputs.isFullyOut;
  }

  public boolean limitSwitch() {
    return inputs.limitSwitch;
  }

  public boolean climberIsFullyIn() {
    return inputs.climberAt0deg;
  }

  public boolean clomberisFullyOut() {
    return inputs.climberAt90deg;
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public boolean isAtMin() {
    if (limitSwitch()) {
      io.resetEncoder();
      return true;
    } else {
      return false;
    }
  }

  public boolean reachedClimbedPosition() {
    return getTareAdjustedPosition() >= ClimberConstants.CLIMBED_POSITION;
  }

  public boolean reachedExtendedPosition() {
    return getTareAdjustedPosition() <= ClimberConstants.EXTENDED_POSITION;
  }

  public double getTareAdjustedPosition() {
    return io.getTareAdjustedPosition();
  }

  public static final double FULL_SPEED = 1.0;
  public static final double HALF_SPEED = 0.5;
  public static final double QUARTER_SPEED = 0.25;

  public Command getClimberInCommand() {
    return getClimberInCommand(FULL_SPEED);
  }

  public Command getClimberInCommand(double manual_multiplier) {
    // Supplier<Boolean> targetReached = () -> this.getPosition() >=
    // ClimberConstants.CLIMBED_POSITION;
    // This is for climbing, and requires the servo to be locked (it's in the same direction as the
    // Ratchet, so no delay needed.)
    if (manual_multiplier < FULL_SPEED || Robot.TEST_MODE) {
      // when in manual mode, just keep it unlocked. we don't need to lock it when we're testing.
      return runPercent(ClimberConstants.IN_SPEED * manual_multiplier, Climber.SERVO_UNLOCKED);
    }

    return runPercent(ClimberConstants.IN_SPEED, Climber.SERVO_LOCKED)
        .until(this::reachedClimbedPosition);
  }

  public void tare() {
    io.tare();
  }

  public Command getClimberOutCommand(Ramp ramp) {
    return getClimberOutCommand(ramp, FULL_SPEED);
  }

  public Command getClimberOutCommand(Ramp ramp, double manual_multiplier) {
    // TODO: We can improve on this by measuring the total time we've spent running the servo to
    // position (reset by going the other way)

    // If manual_multiplier is not FULL_SPEED, it must mean we don't want to trust the
    // reachedExtendedPosition
    Supplier<Double> remainingServoTime = () -> remainingTimeForRatchet();
    if (manual_multiplier < FULL_SPEED || Robot.TEST_MODE) {
      return runServoToPosition(Climber.SERVO_UNLOCKED)
          .withTimeout(
              // we wait to make sure we're not fighting the ratchet, then we continue
              remainingServoTime.get())
          // .withTimeout(TIME_FOR_SERVO_TO_UNLOCK) // we wait to make sure we're not fighting the
          .andThen(
              runPercent(ClimberConstants.OUT_SPEED * manual_multiplier, Climber.SERVO_UNLOCKED));
    }

    return runServoToPosition(Climber.SERVO_UNLOCKED)
        .withTimeout(
            // we wait to make sure we're not fighting the ratchet, then we continue
            remainingServoTime.get())
        // .withTimeout(TIME_FOR_SERVO_TO_UNLOCK) // we wait to make sure we're not fighting the
        .andThen(runPercent(ClimberConstants.OUT_SPEED, Climber.SERVO_UNLOCKED))
        .until(this::reachedExtendedPosition)
        .andThen(
            ramp.runServoAtSpeed(
                Ramp.SERVO_UNLATCH)); // this will unhitch the "wings", and let us climb
  }
}
