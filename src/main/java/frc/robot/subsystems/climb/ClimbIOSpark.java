// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.subsystems.climb.ClimbConstants.climbCanId;
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
public class ClimbIOSpark implements ClimbIO {
  private final SparkFlex climbFlex = new SparkFlex(climbCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = climbFlex.getEncoder();
  //private PIDController pid = new PIDController(1.4, 0, 0);

  public ClimbIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    
    tryUntilOk(
        climbFlex,
        5,
        () ->
            climbFlex.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    ifOk(climbFlex, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(climbFlex, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        climbFlex,
        new DoubleSupplier[] {climbFlex::getAppliedOutput, climbFlex::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(climbFlex, climbFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    climbFlex.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    climbFlex.set(speed);
  }

  // @Override
  // public void runWristPIDController(double sensor, double setPoint) {
  //   wristFlex.set(pid.calculate(sensor, setPoint));
  // }
}
