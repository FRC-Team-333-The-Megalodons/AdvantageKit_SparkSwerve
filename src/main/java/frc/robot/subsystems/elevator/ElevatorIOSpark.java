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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkFlex elevatorMotorTop =
      new SparkFlex(elevatorMotorTopCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorLeft =
      new SparkFlex(elevatorMotorLeftCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorRight =
      new SparkFlex(elevatorMotorRightCanId, MotorType.kBrushless);

  private final RelativeEncoder encoder1 = elevatorMotorTop.getEncoder();
  private PIDController PID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private PIDController PIDL4 = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kP);
  private ElevatorFeedforward FF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  // 0, 0.11, 2.66, 0.05    0, 0.6, 1.33, 0.025

  private final RelativeEncoder encoder2 = elevatorMotorLeft.getEncoder();
  private final RelativeEncoder encoder3 = elevatorMotorRight.getEncoder();

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
        elevatorMotorTop,
        5,
        () ->
            elevatorMotorTop.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        elevatorMotorLeft,
        5,
        () ->
            elevatorMotorLeft.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.follow(elevatorMotorTop);

    tryUntilOk(
        elevatorMotorRight,
        5,
        () ->
            elevatorMotorRight.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.follow(elevatorMotorLeft);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(elevatorMotorTop, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotorTop, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorTop,
        new DoubleSupplier[] {elevatorMotorTop::getAppliedOutput, elevatorMotorTop::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorTop,
        elevatorMotorTop::getOutputCurrent,
        (value) -> inputs.currentAmps = value);

    ifOk(elevatorMotorLeft, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotorLeft, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorLeft,
        new DoubleSupplier[] {
          elevatorMotorLeft::getAppliedOutput, elevatorMotorLeft::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorLeft,
        elevatorMotorLeft::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotorTop.setVoltage(volts);
    elevatorMotorLeft.setVoltage(volts);
    elevatorMotorRight.setVoltage(volts);
  }

  @Override
  public void setValue(double value) {
    encoder1.setPosition(value);
  }

  @Override
  public void runElevatorPIDController(double setPoint) {
    elevatorMotorTop.set(PID.calculate(encoder1.getPosition(), setPoint));
    elevatorMotorLeft.set(PID.calculate(encoder2.getPosition(), setPoint));
    elevatorMotorRight.set(PID.calculate(encoder3.getPosition(), setPoint));
  }

  @Override
  public void runElevatorPIDFFController(double setPoint) {
    elevatorMotorTop.set(
        PID.calculate(encoder1.getPosition(), setPoint)
            + FF.calculate(setPoint));
    elevatorMotorLeft.set(
        PID.calculate(encoder2.getPosition(), setPoint)
            + FF.calculate(setPoint));
    elevatorMotorRight.set(
        PID.calculate(encoder3.getPosition(), setPoint)
            + FF.calculate(setPoint));
  }

  @Override
  public double getPosition() {
    return encoder1.getPosition();
  }

  @Override
  public Object getDistance() {
    throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
  }

  @Override
  public boolean atSetpoint() {
    return PID.atSetpoint();
  }
}
