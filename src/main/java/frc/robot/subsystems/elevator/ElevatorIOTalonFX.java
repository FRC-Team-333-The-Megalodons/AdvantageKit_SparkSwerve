// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

/** This implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX topElevatorMotor = new TalonFX(toplElevatorMotorCanId, "canivore");
  private final TalonFX leftElevatorMotor = new TalonFX(leftElevatorMotorCanId, "canivore");
  private final TalonFX rightElevatorMotor = new TalonFX(rightElevatorMotorCanId, "canivore");
  private final StatusSignal<Angle> positionRot = topElevatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = topElevatorMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = topElevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = topElevatorMotor.getSupplyCurrent();
  private DigitalInput lowerLimitSwitch = new DigitalInput(lowerLimitSwitchId);
  private DigitalInput upperLimitSwitch = new DigitalInput(upperLimitSwitchId);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = ElevatorConstants.kP_CTRE;
    config.Slot0.kI = ElevatorConstants.kI_CTRE;
    config.Slot0.kD = ElevatorConstants.kD_CTRE;
    // config.Slot0.kS = ElevatorConstants.kS_CTRE;
    // config.Slot0.kV = ElevatorConstants.kV_CTRE;

    tryUntilOk(5, () -> topElevatorMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leftElevatorMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightElevatorMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    topElevatorMotor.optimizeBusUtilization();

    topElevatorMotor.setPosition(0);
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    leftElevatorMotor.setControl(new Follower(toplElevatorMotorCanId, false));
    rightElevatorMotor.setControl(new Follower(toplElevatorMotorCanId, false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.position = positionRot.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.lowerLimit = !lowerLimitSwitch.get();
    inputs.upperLimit = !upperLimitSwitch.get();
    inputs.atL4Setpoint =
        inputs.position > ElevatorConstants.closeToL4 - 1
            && inputs.position < ElevatorConstants.closeToL4 + 1;
  }

  @Override
  public void setVoltage(double volts) {
    topElevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setElevator(double currentPos, double targetPos, boolean down) {
    if (down) {
      topElevatorMotor.setControl(positionRequest.withPosition(targetPos));
    } else {
      topElevatorMotor.setControl(positionRequest.withPosition(targetPos));
    }
  }

  @Override
  public void resetEncoder() {
    topElevatorMotor.setPosition(0.0);
  }
}
