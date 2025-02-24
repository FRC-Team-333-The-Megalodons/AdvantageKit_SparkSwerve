// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
<<<<<<< HEAD
import frc.robot.subsystems.LEDStrip;
=======
>>>>>>> 2e6a5294c865f61289f7355695aa6832bd26136e
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class GoRemoveAlgaeL3 extends SequentialCommandGroup {
  public GoRemoveAlgaeL3(Intake intake, Wrist wrist, Elevator elevator /* , LEDStrip ledStrip*/) {
    addCommands(
<<<<<<< HEAD
        new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue)),
        wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L3_POS),
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS),
        intake
            .runPercent(0.5)
            .until(intake::isTriggered)
            .alongWith(new RunCommand(() -> LEDStrip.setLEDs(Color.kGreen)))
            );
=======
        //  ledStrip.setColor(LEDColor.RED),
        wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L3_POS).until(wrist::atSetpoint),
        elevator
            .setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS)
            .until(elevator::atSetpoint),
        intake.runPercent(0.5).until(intake::isTriggered)
        //      .alongWith(ledStrip.setColor(LEDColor.GREEN))
        );
>>>>>>> 2e6a5294c865f61289f7355695aa6832bd26136e
  }
}
