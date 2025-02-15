// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class WristIOSpark implements WristIO {
  private final SparkFlex wrist = new SparkFlex(wristCanId, MotorType.kBrushless);
  private final RelativeEncoder internalEncoder = wrist.getEncoder();
  private final DutyCycleEncoder externalEncoder = new DutyCycleEncoder(wristEncoderId);
  private final PIDController pid = new PIDController(1.5, 0.0, 0.0);

  public WristIOSpark() {
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
        wrist,
        5,
        () ->
            wrist.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    ifOk(wrist, internalEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        wrist,
        new DoubleSupplier[] {wrist::getAppliedOutput, wrist::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(wrist, wrist::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(wrist, externalEncoder::get, (value) -> inputs.positionAbs = value);
  }

  @Override
  public void setVoltage(double volts) {
    wrist.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    wrist.set(speed);
  }

  @Override
  public void setWrist(double currentPos, double targetPos) {
    wrist.set(-pid.calculate(currentPos, targetPos));
  }

  @Override
  public boolean atTarget() {
    return pid.atSetpoint();
  }
}
