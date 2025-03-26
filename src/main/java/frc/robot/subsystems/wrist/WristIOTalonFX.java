// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class WristIOTalonFX implements WristIO {
  private final TalonFX wrist = new TalonFX(wristCanId);
  private final StatusSignal<Angle> positionRot = wrist.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = wrist.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = wrist.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = wrist.getSupplyCurrent();
  private final DutyCycleEncoder externalEncoder = new DutyCycleEncoder(wristEncoderId);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PIDController pidController = new PIDController(kP, kI, kD);

  public WristIOTalonFX() {
    var config = new TalonFXConfiguration();

    config.Slot0.kP = WristConstants.kP_CTRE;
    config.Slot0.kI = WristConstants.kI_CTRE;
    config.Slot0.kD = WristConstants.kD_CTRE;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> wrist.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    wrist.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    inputs.positionAbs = externalEncoder.get();
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.atSetpoint = pidController.atSetpoint();
    inputs.atL4Setpoint = inputs.positionAbs > 0.35 && inputs.positionAbs < 0.39 ? true : false;
    inputs.atHomePosition = inputs.positionAbs > 0.53 && inputs.positionAbs < 0.55 ? true : false;
    inputs.atAlgaeSetpoint = inputs.positionAbs > 0.11 && inputs.positionAbs < 0.13 ? true : false;
    inputs.atNetSetPoint = inputs.positionAbs > 0.39 && inputs.positionAbs < 0.42 ? true : false;
  }

  @Override
  public void setVoltage(double volts) {
    wrist.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setWristPosition(double currentPos, double targetPos) {
    wrist.set(-pidController.calculate(currentPos, targetPos));
    // to be worked on later
    // wrist.setControl(positionVoltage.withPosition(targetPos));
  }
}
