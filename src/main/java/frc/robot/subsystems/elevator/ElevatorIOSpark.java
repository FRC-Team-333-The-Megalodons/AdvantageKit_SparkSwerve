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
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkFlex elevatorMotor1 = new SparkFlex(elevatorMotor1CanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotor2 = new SparkFlex(elevatorMotor2CanId, MotorType.kBrushless);
  private final RelativeEncoder encoder1 = elevatorMotor1.getEncoder();
  private final RelativeEncoder encoder2 = elevatorMotor2.getEncoder();
  private final PIDController pidController = new PIDController(0.5, 0.0, 0.0);

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
        elevatorMotor1,
        5,
        () ->
            elevatorMotor1.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        elevatorMotor2,
        5,
        () ->
            elevatorMotor2.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Elevator Motors
    ifOk(elevatorMotor1, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotor1, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotor1,
        new DoubleSupplier[] {elevatorMotor1::getAppliedOutput, elevatorMotor1::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(elevatorMotor1, elevatorMotor1::getOutputCurrent, (value) -> inputs.currentAmps = value);

    ifOk(elevatorMotor2, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotor2, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotor2,
        new DoubleSupplier[] {elevatorMotor2::getAppliedOutput, elevatorMotor2::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(elevatorMotor2, elevatorMotor2::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotor1.setVoltage(volts);
    elevatorMotor2.setVoltage(volts);
  }

  @Override
  public void setElevator(double currentPos, double targetPos) {
    elevatorMotor1.set(pidController.calculate(currentPos, targetPos));
  }
}
