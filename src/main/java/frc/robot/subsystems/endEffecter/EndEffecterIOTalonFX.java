// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffecter;

import static frc.robot.subsystems.endEffecter.EndEffecterConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class EndEffecterIOTalonFX implements EndEffecterIO {
  private final TalonFX endeffecter = new TalonFX(endEffecterCanId);
  private final StatusSignal<Angle> positionRot = endeffecter.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = endeffecter.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = endeffecter.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = endeffecter.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public EndEffecterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> endeffecter.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    endeffecter.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EndEffecterIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    endeffecter.setControl(voltageRequest.withOutput(volts));
  }
}
