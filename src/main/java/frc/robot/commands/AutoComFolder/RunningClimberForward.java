// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoComFolder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;

public class RunningClimberForward extends SequentialCommandGroup {
  public RunningClimberForward(Climb climb /* , LEDStrip ledStrip*/) {

    addCommands(
        // ledStrip.setColor(LEDColor.BLUE),
        climb.runPercent(1)
        // .alongWith(ledStrip.setColor(LEDColor.ORANGE))
        );
  }
}
