// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFX wrist;
  public Wrist() {
    wrist = new TalonFX(4);
  }

  public void setSpeed(double speed) {
    wrist.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
