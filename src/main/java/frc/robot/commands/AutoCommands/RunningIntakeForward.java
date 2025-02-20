// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.LEDColor;
import frc.robot.subsystems.intake.Intake;

public class RunningIntakeForward extends SequentialCommandGroup {
  public RunningIntakeForward(Intake intake, LEDStrip ledStrip) {

    addCommands(
        ledStrip.setColor(LEDColor.BLUE),
        intake
        .runPercent(0.5)
        .until(intake::isTriggered)
        .alongWith(ledStrip.setColor(LEDColor.ORANGE)));
  }
}
