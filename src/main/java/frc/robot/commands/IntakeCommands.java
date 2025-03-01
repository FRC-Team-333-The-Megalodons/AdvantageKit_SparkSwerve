// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.intake.IntakeConstants.speed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

  private IntakeCommands() {}

  public static Command runIntakeForward(Intake Intake) {
    return Intake.runPercent(speed);
  }

  public static Command runIntakeBackward(Intake Intake) {
    return Intake.runPercent(-speed);
  }
}
