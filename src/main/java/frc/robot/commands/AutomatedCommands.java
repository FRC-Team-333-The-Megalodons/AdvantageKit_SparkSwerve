// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endEffecter.EndEffecterConstants;
import frc.robot.subsystems.endEffecter.EndEffector;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

/** Add your docs here. */
public class AutomatedCommands {

  public static Command intakeCoral(EndEffector endEffecter, LEDStrip led) {
    return endEffecter
        .runPercent(EndEffecterConstants.speed)
        .until(endEffecter::isTriggered)
        .andThen(led.makeWholeColorCommand(Color.kGreen));
  }

  public static Command homeCommand(Wrist wrist, Elevator elevator, LEDStrip led) {
    return elevator
        .setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION)
        .until(elevator::atSetpoint)
        .andThen(wrist.setWristPosition(WristConstants.WRIST_HOME_POSITION));
  }

  public static Command coralL4Command(
      EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L2_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS))
        .until(elevator::atSetpoint)
        .andThen(
            wrist
                .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L4_POS)
                .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS)));
  }

  public static Command coralL3Command(
      EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L3_POS));
  }

  public static Command coralL2Command(
    EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L2_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L2_POS));
  }

  public static Command algaeL3Command(
    EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L3_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS));
  }

  public static Command algaeL2Command(
    EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L2_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L2_POS));
  }

  public static Command processorCommand(
      EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_ALGAE_SCORE_PROCESSOR_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_SCORE_PROCESSOR_POS));
  }

  public static Command homeWithAlgaeCommand(
      EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_HOME_POSITION)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION));
  }

  public static Command netCommand(
      EndEffector endEffecter, Wrist wrist, Elevator elevator, LEDStrip led) {
    return wrist
        .setWristPosition(WristConstants.WRIST_ALGAE_SCORE_NET_POS)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_SCORE_NET_POS));
  }
}
