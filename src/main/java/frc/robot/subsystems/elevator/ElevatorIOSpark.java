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
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkFlex elevator = new SparkFlex(elevatorMotor1CanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = elevator.getEncoder();

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
        elevator,
        5,
        () ->
            elevator.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(elevator, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevator, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevator,
        new DoubleSupplier[] {elevator::getAppliedOutput, elevator::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(elevator, elevator::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    elevator.setVoltage(volts);
  }
}
