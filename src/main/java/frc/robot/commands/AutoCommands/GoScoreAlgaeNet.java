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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoScoreAlgaeNet extends SequentialCommandGroup {
  public GoScoreAlgaeNet(Intake intake, Wrist wrist, Elevator elevator, LEDStrip ledStrip) {
    addCommands(
        new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue)),
        wrist.setWristPosition(WristConstants.WRIST_ALGAE_SCORE_NET_POS).until(wrist::atSetpoint),
        elevator
            .setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_SCORE_NET_POS)
            .until(elevator::atSetpoint),
        intake
            .runPercent(0.5)
            .until(intake::isTriggered)
            .alongWith(new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue))));
  }
}
