// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffecter;

import static frc.robot.subsystems.endEffecter.EndEffecterConstants.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class EndEffecterIOSim implements EndEffecterIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.004, motorReduction),
          DCMotor.getNeoVortex(1));

  private double appliedVolts = 0.0;
  private double speed = Double.MIN_VALUE;

  private CANrange caNrange = new CANrange(canRangeId);
  private CANrangeSimState tofSimState = caNrange.getSimState();

  @Override
  public void updateInputs(EndEffecterIOInputs inputs) {

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
