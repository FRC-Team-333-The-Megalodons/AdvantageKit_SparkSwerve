// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endEffecter.EndEffecter;
import frc.robot.subsystems.endEffecter.EndEffecterConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

/** Add your docs here. */
public class AutomatedCommands {

  public static Command intakeCoral(EndEffecter endEffecter, LEDStrip led) {
    return endEffecter
        .runPercent(EndEffecterConstants.speed)
        .until(endEffecter::isTriggered)
        .andThen(led.makeWholeColorCommand(Color.kGreen));
  }

  public static Command homeCommand(Wrist wrist, Elevator elevator, LEDStrip led) {
    return elevator
        .setElevatorPosition(ElevatorConstants.homeSetpoint, true)
        .until(elevator::lowerLimit)
        .andThen(wrist.setWristPosition(WristConstants.homeSetpoint));
  }

  public static Command coralL4Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, false))
        .until(elevator::atL4Setpoint)
        .andThen(
            wrist
                .setWristPosition(WristConstants.coralL4Setpoint)
                .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, false)));
  }

  public static Command coralL3Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL3Setpoint, true));
  }

  public static Command coralL2Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL2Setpoint, true));
  }

  public static Command algaeL3Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.aglaeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.aglaeL3Setpoint, true));
  }

  public static Command algaeL2Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.aglaeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint, true));
  }

  public static Command processorCommand(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.processorSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.processorSetpoint, true));
  }

  public static Command homeWithAlgaeCommand(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.algaeHomeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.homeSetpoint, true));
  }

  public static Command netCommand(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.netSetPoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.bargeSetPoint, true));
  }
}
