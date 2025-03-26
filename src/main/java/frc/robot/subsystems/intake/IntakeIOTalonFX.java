// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  /** Creates a new IntakeIOTalonFX. */
  private TalonFX intakeTalon = new TalonFX(16);

  private TalonFXConfiguration newMagicMotion = new TalonFXConfiguration();

  public IntakeIOTalonFX() {
    var configure = new TalonFXConfiguration();

    var slot0Configs = newMagicMotion.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.17; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.9; // A velocity error of 1 rps results in 0.1 V output

    intakeTalon.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void runWristPIDController(double sensor, double setPoint) {
    intakeTalon.setPosition(65);
  }
}
