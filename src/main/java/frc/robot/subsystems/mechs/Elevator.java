// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {
    LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
    Logger.recordOutput("Elevator", mechanism);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
