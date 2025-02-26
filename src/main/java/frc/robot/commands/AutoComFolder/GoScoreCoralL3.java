// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoComFolder;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endEffecter.EndEffector;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class GoScoreCoralL3 extends SequentialCommandGroup {
  public GoScoreCoralL3(EndEffector intake, Wrist wrist, Elevator elevator, LEDStrip ledStrip) {

    addCommands(
        new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue)),
        wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS),
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L3_POS),
        new RunCommand(() -> LEDStrip.setLEDs(Color.kGreen)));
  }
}
