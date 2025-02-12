// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.LEDColor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoRemoveAlgaeL2 extends SequentialCommandGroup {
  /** Creates a new GoRemoveAlgaeL2. */
  public GoRemoveAlgaeL2(Intake intake, Wrist wrist, Elevator elevator, LEDStrip ledStrip) {
    addCommands(
      ledStrip.setColor(LEDColor.RED),
      wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L2_POS),
      elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L2_POS),
      intake.runPercent(0.5).until(intake::isTriggered).alongWith(ledStrip.setColor(LEDColor.GREEN))
    );
  }
}
