// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkFlex topElevatorMotor =
      new SparkFlex(toplElevatorMotorCanId, MotorType.kBrushless);
  private final SparkFlex leftElevatorMotor =
      new SparkFlex(leftElevatorMotorCanId, MotorType.kBrushless);
  private final SparkFlex rightElevatorMotor =
      new SparkFlex(rightElevatorMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = topElevatorMotor.getEncoder();
  private final PIDController elevatorUpPidController = new PIDController(0.012, 0.0, 0.0005);
  private final PIDController elevatorDownPidController = new PIDController(0.006, 0.0, 0.0);
  private DigitalInput lowerLimitSwitch = new DigitalInput(lowerLimitSwitchId);
  private DigitalInput upperLimitSwitch = new DigitalInput(upperLimitSwitchId);

  public ElevatorIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> intake Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        topElevatorMotor,
        5,
        () ->
            topElevatorMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        leftElevatorMotor,
        5,
        () ->
            leftElevatorMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Elevator Motors
    ifOk(topElevatorMotor, encoder::getPosition, (value) -> inputs.position = value);
    ifOk(
        topElevatorMotor,
        new DoubleSupplier[] {topElevatorMotor::getAppliedOutput, topElevatorMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        topElevatorMotor,
        topElevatorMotor::getOutputCurrent,
        (value) -> inputs.currentAmps = value);

    inputs.atSetpoint =
        elevatorDownPidController.atSetpoint() || elevatorDownPidController.atSetpoint();
    inputs.lowerLimit = !lowerLimitSwitch.get();
    inputs.upperLimit = !upperLimitSwitch.get();
    inputs.atL4Setpoint = getElevatorPosition() > 190 ? true : false;
  }

  @Override
  public void setVoltage(double volts) {
    topElevatorMotor.setVoltage(volts);
    leftElevatorMotor.setVoltage(volts);
    rightElevatorMotor.setVoltage(volts);
  }

  @Override
  public void setElevator(double currentPos, double targetPos, boolean down) {
    if (down) {
      topElevatorMotor.set(elevatorDownPidController.calculate(currentPos, targetPos));
      leftElevatorMotor.set(elevatorDownPidController.calculate(currentPos, targetPos));
      rightElevatorMotor.set(elevatorDownPidController.calculate(currentPos, targetPos));
    } else {
      topElevatorMotor.set(elevatorUpPidController.calculate(currentPos, targetPos));
      leftElevatorMotor.set(elevatorUpPidController.calculate(currentPos, targetPos));
      rightElevatorMotor.set(elevatorUpPidController.calculate(currentPos, targetPos));
    }
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public double getElevatorPosition() {
    return encoder.getPosition();
  }
}
