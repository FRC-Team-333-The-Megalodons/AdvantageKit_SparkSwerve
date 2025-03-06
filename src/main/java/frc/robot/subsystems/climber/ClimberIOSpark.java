// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
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
public class ClimberIOSpark implements ClimberIO {
  private final SparkFlex climber = new SparkFlex(climberCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = climber.getEncoder();
  // private final DigitalInput limitSwitch = new DigitalInput(limitSwitchId);

  public ClimberIOSpark() {
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
        climber,
        5,
        () ->
            climber.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(climber, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(climber, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        climber,
        new DoubleSupplier[] {climber::getAppliedOutput, climber::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(climber, climber::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.isFullyIn = inputs.positionRad == ClimberConstants.fullyIn;
    inputs.isFullyOut = inputs.positionRad > 0.5 && inputs.positionRad < 0.52 ? true : false;

    // inputs.limitSwitch = !limitSwitch.get();
  }

  @Override
  public void setVoltage(double volts) {
    climber.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    climber.set(speed);
  }
  
}
