// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
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
public class HopperIOSpark implements HopperIO {
  private final SparkFlex hopper = new SparkFlex(hopperCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = hopper.getEncoder();

  public HopperIOSpark() {
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
        hopper,
        5,
        () ->
            hopper.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    ifOk(hopper, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(hopper, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        hopper,
        new DoubleSupplier[] {hopper::getAppliedOutput, hopper::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(hopper, hopper::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    hopper.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    hopper.set(speed);
  }
}
