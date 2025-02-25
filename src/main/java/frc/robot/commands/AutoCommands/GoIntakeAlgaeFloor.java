// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class GoIntakeAlgaeFloor extends SequentialCommandGroup {
  public GoIntakeAlgaeFloor(Intake intake, Wrist wrist, Elevator elevator, LEDStrip ledStrip) {

    addCommands(
        new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue)),
        wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_FLOOR_POS),
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_FLOOR_POS),
        intake
            .runPercent(0.5)
            .until(intake::isTriggered)
            .alongWith(new RunCommand(() -> LEDStrip.setLEDs(Color.kOrange))));
  }
}
