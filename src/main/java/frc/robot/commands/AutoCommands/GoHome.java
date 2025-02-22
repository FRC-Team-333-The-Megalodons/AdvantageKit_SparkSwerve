// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHome extends SequentialCommandGroup {
  public GoHome(Wrist wrist, Elevator elevator, LEDStrip ledStrip) {

    addCommands(
        new RunCommand(() -> LEDStrip.setLEDs(Color.kBlue)),
        wrist.setWristPosition(WristConstants.WRIST_HOME_POSITION),
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION),
        new RunCommand(() -> LEDStrip.setLEDs(Color.kOrange)));
  }
}
