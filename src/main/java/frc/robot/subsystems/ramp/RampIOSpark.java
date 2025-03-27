// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

import static frc.robot.subsystems.ramp.RampConstants.*;
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
public class RampIOSpark implements RampIO {
  private final SparkFlex ramp = new SparkFlex(rampCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = ramp.getEncoder();
  private final PIDController pidController = new PIDController(kP, kI, kD);
  // private final DigitalInput digitalInputSensor = new DigitalInput(rampLimitSwitch1Id);
  // private final DigitalInput limitSwitch2 = new DigitalInput(rampLimitSwitch2Id);

  public RampIOSpark() {
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
        ramp,
        5,
        () ->
            ramp.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(RampIOInputs inputs) {
    ifOk(ramp, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(ramp, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        ramp,
        new DoubleSupplier[] {ramp::getAppliedOutput, ramp::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(ramp, ramp::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.atSetpoint = pidController.atSetpoint();
    // inputs.digitalInputSensor = digitalInputSensor.get();
    // inputs.limitSwitch2 = limitSwitch2.get();
  }

  @Override
  public void setVoltage(double volts) {
    ramp.setVoltage(volts);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0.0);
  }

  @Override
  public void setSpeed(double speed) {
    ramp.set(speed);
  }

  @Override
  public void setRampPosition(double currentPos, double targetPos) {
    ramp.setVoltage(pidController.calculate(currentPos, targetPos));
  }
}
