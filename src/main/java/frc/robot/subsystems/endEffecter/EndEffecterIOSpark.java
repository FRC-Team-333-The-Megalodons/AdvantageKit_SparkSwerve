// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffecter;

import static frc.robot.subsystems.endEffecter.EndEffecterConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class EndEffecterIOSpark implements EndEffecterIO {
  private final SparkFlex endEffecter = new SparkFlex(endEffecterCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = endEffecter.getEncoder();
  private final CANrange canRange = new CANrange(canRangeId);
  private final DigitalInput photoEletricSensor = new DigitalInput(photoEletricSensorId);

  public EndEffecterIOSpark() {
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
        endEffecter,
        5,
        () ->
            endEffecter.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EndEffecterIOInputs inputs) {
    ifOk(endEffecter, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(endEffecter, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        endEffecter,
        new DoubleSupplier[] {endEffecter::getAppliedOutput, endEffecter::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(endEffecter, endEffecter::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.isTriggered = canRange.getIsDetected().getValue();
    inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();
    inputs.isConnected = canRange.isConnected();
    inputs.hasAlgae = photoEletricSensor.get();
  }

  @Override
  public void setVoltage(double volts) {
    endEffecter.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    endEffecter.set(speed);
  }
}
