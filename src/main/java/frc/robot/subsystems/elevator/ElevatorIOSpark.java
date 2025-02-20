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
  private final SparkFlex elevatorMotorLeader =
      new SparkFlex(elevatorMotorLeaderCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorFollower1 =
      new SparkFlex(elevatorMotorFollowerCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorFollower2 =
      new SparkFlex(elevatorMotorFollower2CanId, MotorType.kBrushless);

  private final RelativeEncoder encoder1 =
      elevatorMotorLeader.getEncoder(); 
  private PIDController elevatorPIDController 
  = new PIDController(0.06, 0.0, 0);
  private ElevatorFeedforward elevatorFFController = 
    new ElevatorFeedforward(0.0, 0.11, 3.6, 0.6);
    // 0, 0.11, 2.66, 0.05    0, 0.6, 1.33, 0.025



  private final RelativeEncoder encoder2 = elevatorMotorFollower1.getEncoder();
  private final RelativeEncoder encoder3 = elevatorMotorFollower2.getEncoder();

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
        elevatorMotorLeader,
        5,
        () ->
            elevatorMotorLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        elevatorMotorFollower1,
        5,
        () ->
            elevatorMotorFollower1.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.follow(elevatorMotorLeader);

    tryUntilOk(
        elevatorMotorFollower2,
        5,
        () ->
            elevatorMotorFollower2.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.follow(elevatorMotorFollower1);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(elevatorMotorLeader, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotorLeader, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorLeader,
        new DoubleSupplier[] {
          elevatorMotorLeader::getAppliedOutput, elevatorMotorLeader::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorLeader,
        elevatorMotorLeader::getOutputCurrent,
        (value) -> inputs.currentAmps = value);

    ifOk(elevatorMotorFollower1, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(
        elevatorMotorFollower1, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorFollower1,
        new DoubleSupplier[] {
          elevatorMotorFollower1::getAppliedOutput, elevatorMotorFollower1::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorFollower1,
        elevatorMotorFollower1::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotorLeader.setVoltage(volts);
    elevatorMotorFollower1.setVoltage(volts);
    elevatorMotorFollower2.setVoltage(volts);
  }

  @Override
  public void setValue(double value) {
    encoder1.setPosition(value);
  }

  @Override
  public void runElevatorPIDController(double setPoint) {
    elevatorMotorLeader.set(elevatorPIDController.calculate(encoder1.getPosition(), setPoint));
    elevatorMotorFollower1.set(elevatorPIDController.calculate(encoder2.getPosition(), setPoint));
    elevatorMotorFollower2.set(elevatorPIDController.calculate(encoder3.getPosition(), setPoint));
  }

  @Override
  public void runElevatorPIDFFController(double setPoint) {
    elevatorMotorLeader.set(
        elevatorPIDController.calculate(encoder1.getPosition(), setPoint)
            + elevatorFFController.calculate(setPoint));
    elevatorMotorFollower1.set(
        elevatorPIDController.calculate(encoder2.getPosition(), setPoint)
            + elevatorFFController.calculate(setPoint));
    elevatorMotorFollower2.set(
        elevatorPIDController.calculate(encoder3.getPosition(), setPoint)
            + elevatorFFController.calculate(setPoint));
  }
  @Override
  public double getPosition() {
    return encoder1.getPosition();
  }
}
