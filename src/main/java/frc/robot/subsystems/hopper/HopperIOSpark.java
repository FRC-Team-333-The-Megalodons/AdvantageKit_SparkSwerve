// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
import static frc.robot.subsystems.hopper.HopperConstants.hopperCanId;
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
  private final SparkFlex hopperFlex = new SparkFlex(hopperCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = hopperFlex.getEncoder();
  // private PIDController pid = new PIDController(1.4, 0, 0);

  public HopperIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);

    tryUntilOk(
        hopperFlex,
        5,
        () ->
            hopperFlex.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    ifOk(hopperFlex, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(hopperFlex, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        hopperFlex,
        new DoubleSupplier[] {hopperFlex::getAppliedOutput, hopperFlex::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(hopperFlex, hopperFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    hopperFlex.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    hopperFlex.set(speed);
  }

  // @Override
  // public void runHopperPIDController(double sensor, double setPoint) {
  //   hopperFlex.set(pid.calculate(sensor, setPoint));
  // }
}
