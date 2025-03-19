// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.endEffecter.EndEffecterConstants.speed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffecter.EndEffecter;

public class EndEffecterCommands {

  private EndEffecterCommands() {}

  public static Command runEndEffecterForward(EndEffecter endEffecter) {
    return endEffecter.runPercent(speed);
  }

  public static Command runEndEffecterBackward(EndEffecter endEffecter) {
    return endEffecter.runPercent(-speed * 2);
  }

  public static Command autoRunEndEffecterForward(EndEffecter endEffecter) {
    return endEffecter.runPercent(speed + 0.25);
  }
}
