// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.LEDColor;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.elevator.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome(Wrist wrist, Elevator elevator, LEDStrip ledStrip) {

  addCommands(
    wrist.setWristPosition(WristConstants.WRIST_HOME_POSITION),
    elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION),
    ledStrip.setColor(LEDColor.BLUE)
  );
  }
}
