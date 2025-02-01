// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.Optional;

import com.fasterxml.jackson.databind.annotation.JsonTypeIdResolver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, motorReduction),
          DCMotor.getNEO(1));

  private double appliedVolts = 0.0;
  private double speed = Double.MIN_VALUE;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    if (speed != Double.MIN_VALUE) {
      sim.setInput(speed);
    } else {
      sim.setInputVoltage(appliedVolts);
    }
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    speed = Double.MIN_VALUE;
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
