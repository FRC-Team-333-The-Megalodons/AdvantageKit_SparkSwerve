// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ramp;

import static frc.robot.subsystems.ramp.RampConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
public class RampIOTalonFX implements RampIO {
  private final TalonFX leftMotor = new TalonFX(RampConstants.leftMotorID, "canivore");
  private final TalonFX rightMotor = new TalonFX(RampConstants.rightMotorID, "canivore");
  private final StatusSignal<Angle> positionRot = rightMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = rightMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = rightMotor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = rightMotor.getSupplyCurrent();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public RampIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    rightMotor.optimizeBusUtilization();

    leftMotor.setControl(new Follower(RampConstants.rightMotorID, true));
  }

  @Override
  public void updateInputs(RampIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    rightMotor.setControl(voltageRequest.withOutput(volts));
  }
}
