// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.elevator.ElevatorConstants;

/** This climber implementation is for a Talon FX driving a Kraken X60. */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climber = new TalonFX(climberCanId, "canivore");
  private final StatusSignal<Angle> positionRot = climber.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = climber.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = climber.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = climber.getSupplyCurrent();
  private final Servo climberServo = new Servo(9); // out a servo port pls

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final PositionDutyCycle poisitionRequest = new PositionDutyCycle(0).withSlot(0);


  public ClimberIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
     config.Slot0.kP = ElevatorConstants.kP_CTRE;

    tryUntilOk(5, () -> climber.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    climber.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = positionRot.getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.isAt90deg = climberServo.getPosition() == 1.0 ? true : false;
    inputs.isAt0deg = climberServo.getPosition() == 0.0 ? true : false;
    inputs.servoPosition = climberServo.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    climber.setControl(voltageRequest.withOutput(volts));
  }
  @Override
  public  void setClimberPos(double currentPos, double tragetPos) {
      climber.setControl(poisitionRequest.withPosition(tragetPos));
  }

  @Override
  public void setSpeedServo(double speed) {
    climberServo.set(speed);
  }
}
