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
import edu.wpi.first.math.controller.PIDController;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class HopperIOSpark implements HopperIO {
  private final SparkFlex motor = new SparkFlex(hopperCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private PIDController pid = new PIDController(1.4, 0, 0);

  public HopperIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void runHopperPIDController(double setPoint) {
    motor.set(pid.calculate(encoder.getPosition(), setPoint));
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }
}
