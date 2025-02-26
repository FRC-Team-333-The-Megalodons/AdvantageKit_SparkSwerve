// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX topElevatorMotor = new TalonFX(toplElevatorMotorCanId);
  private final TalonFX leftElevatorMotor = new TalonFX(leftElevatorMotorCanId);
  private final TalonFX rightElevatorMotor = new TalonFX(rightElevatorMotorCanId);
  private final StatusSignal<Angle> positionRot = topElevatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = topElevatorMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = topElevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = topElevatorMotor.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> topElevatorMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    topElevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    topElevatorMotor.setControl(voltageRequest.withOutput(volts));
    leftElevatorMotor.setControl(voltageRequest.withOutput(volts));
    rightElevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void resetEncoder() {
    topElevatorMotor.setPosition(0.0);
  }
}
