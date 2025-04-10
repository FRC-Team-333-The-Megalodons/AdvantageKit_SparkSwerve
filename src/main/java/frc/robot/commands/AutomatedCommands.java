// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endEffecter.EndEffecter;
import frc.robot.subsystems.endEffecter.EndEffecterConstants;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.ramp.RampConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

/** Add your docs here. */
public class AutomatedCommands {

  public static Command intakeCoral(EndEffecter endEffecter) {
    return endEffecter.runPercent(EndEffecterConstants.speed).until(endEffecter::isTriggered);
  }

  // public static Command intakeCoral(EndEffecter endEffecter) {
  //   return endEffecter
  //       .runPercent(EndEffecterConstants.speed)
  //       .until(endEffecter::isTriggered);
  //       // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint));
  // }

  public static Command intakeCoralAgain(EndEffecter endEffecter) {
    return endEffecter.runPercent(EndEffecterConstants.speed).until(endEffecter::isTriggered);
    // .alongWith(ramp.setRampPosition(RampConstants.intakeSetpoint));
  }

  public static Command rampGoToIntakePosition(EndEffecter endEffecter) {
    return endEffecter.runPercent(EndEffecterConstants.speed).until(endEffecter::isTriggered);
    // .alongWith(ramp.setRampPosition(RampConstants.intakeSetpoint));
  }

  public static Command homeCommand(Wrist wrist, Elevator elevator, Ramp ramp) {
    return elevator
        .setElevatorPosition(ElevatorConstants.homeSetpoint, false)
        .until(elevator::lowerLimit)
        .andThen(wrist.setWristPosition(WristConstants.homeSetpoint))
        .alongWith(rampIntakeCommand(ramp, RampConstants.speed));
    // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint)));
  }

  public static Command coralL4Command(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .until(wrist::atL3Setpoint)
        .andThen(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, false))
        // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint))
        .until(elevator::atL4Setpoint)
        .andThen(
            wrist
                .setWristPosition(WristConstants.coralL4Setpoint)
                .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, false)));
  }

  public static Command coralL3Command(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL3Setpoint, false));
    // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint));
  }

  public static Command coralL2Command(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL2Setpoint, false));
    // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint));
  }

  public static Command algaeL3Command(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.aglaeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.aglaeL3Setpoint, false));
  }

  public static Command algaeL2Command(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.aglaeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint, false));
  }

  public static Command processorCommand(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.processorSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.processorSetpoint, true));
  }

  public static Command homeWithAlgaeCommand(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.algaeHomeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.homeSetpoint, true));
  }

  public static Command netCommand(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.netSetPoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.netSetPoint, false));
  }

  // public static Command netLobCommand(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
  //   return EndEffecterCommands.runEndEffecterBackward(endEffecter)
  //       .alongWith(elevator.setElevatorPosition(ElevatorConstants.netSetPoint, false))
  //       .until(elevator::atL4Setpoint)
  //       .andThen(wrist.setWristPosition(WristConstants.netLobSetPoint))
  //       .until(wrist::atNetLobSetPoint)
  //       .andThen(EndEffecterCommands.runEndEffecterForward(endEffecter));
  // }

  public static Command netLobCommand(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return EndEffecterCommands.runEndEffecterBackward(endEffecter)
        .alongWith(wrist.setWristPosition(WristConstants.netLobSetPoint))
        .until(wrist::atNetLobSetPoint)
        .andThen(elevator.setElevatorPosition(ElevatorConstants.netSetPoint, false))
        .until(elevator::atL4Setpoint)
        .andThen(EndEffecterCommands.runEndEffecterForward(endEffecter));
  }

  public static Command rampIntakeCommand(Ramp ramp, double percent) {
    return ramp.runPercent(percent);
  }

  public static Command scoreCoral(EndEffecter endEffecter, Wrist wrist) {

    return EndEffecterCommands.runEndEffecterForward(endEffecter)
        .onlyWhile(endEffecter::isTriggered)
        .onlyWhile(wrist::atL4Setpoint);
  }

  /* Autonomous Commands */

  public static Command autoScoreL4(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.coralL23Setpoint)
        // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint))
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, false))
        .until(elevator::atL4Setpoint)
        .andThen(wrist.setWristPosition(WristConstants.coralL4Setpoint).until(wrist::atL4Setpoint));
  }

  public static Command autoIntakeCoral(EndEffecter endEffecter) {
    // return ramp.setRampPosition(RampConstants.intakeSetpoint)
    // .onlyWhile(ramp::isCoralInside)
    return EndEffecterCommands.autoRunEndEffecterForward(endEffecter)
        .until(endEffecter::isTriggered);
  }

  public static Command autoHomeCommand(Wrist wrist, Elevator elevator) {
    return elevator
        .setElevatorPosition(ElevatorConstants.homeSetpoint, false)
        // .alongWith(ramp.setRampPosition(RampConstants.coralStationSetpoint))
        .until(elevator::lowerLimit)
        .andThen(wrist.setWristPosition(WristConstants.homeSetpoint).until(wrist::atHomePosition));
  }

  public static Command autoAlgaeL2Command(
      EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return wrist
        .setWristPosition(WristConstants.aglaeSetpoint)
        .alongWith(elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint, false))
        .withTimeout(3);
  }

  public static Command autoNetCommand(EndEffecter endEffecter, Wrist wrist, Elevator elevator) {
    return EndEffecterCommands.runEndEffecterBackward(endEffecter)
        .alongWith(
            wrist
                .setWristPosition(WristConstants.netSetPoint)
                .alongWith(elevator.setElevatorPosition(ElevatorConstants.netSetPoint, false)));
  }
}
