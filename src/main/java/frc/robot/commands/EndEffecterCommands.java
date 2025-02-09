// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.endEffecter.EndEffecter;

public class EndEffecterCommands {

  private EndEffecterCommands() {}

  public static Command intakeWithSensor(EndEffecter endEffecter, double seconds) {
    return Commands.run(
        () -> {
          if (!endEffecter.isTriggered()) {
            endEffecter.runPercent(0.5);
          } else {
            new WaitCommand(seconds).andThen(endEffecter.runPercent(0.0));
          }
        },
        endEffecter);
  }
}
