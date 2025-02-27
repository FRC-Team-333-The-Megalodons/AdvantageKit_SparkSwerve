// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import static frc.robot.subsystems.endEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class EndEffectorIOSpark implements EndEffectorIO {
  private final SparkFlex endEffector = new SparkFlex(endEffectorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = endEffector.getEncoder();

  public EndEffectorIOSpark() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> endEffector Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        endEffector,
        5,
        () ->
            endEffector.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) { 
    ifOk(endEffector, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(endEffector, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        endEffector,
        new DoubleSupplier[] {endEffector::getAppliedOutput, endEffector::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(endEffector, endEffector::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    endEffector.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    endEffector.set(speed);
  }
}
