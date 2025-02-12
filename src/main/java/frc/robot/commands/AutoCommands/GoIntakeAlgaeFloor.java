// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOSpark;

public class GoIntakeAlgaeFloor extends SequentialCommandGroup {
  /** Creates a new GoIntakeAlgaeFloor. */
  public GoIntakeAlgaeFloor(Intake intake, IntakeIOSpark intakeIOSpark, LEDStrip ledStrip) {

    addCommands(
      intake.runPercent(IntakeConstants.intakeForwardSpeed).until(() -> intake.isTriggered()).alongWith(ledStrip.setColor(LEDStrip.LEDColor.GREEN))
    );

  }
}
