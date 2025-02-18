package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class IntakeCommands {

  private IntakeCommands() {}

  // public static Command intakeWithSensor(Intake intake, double seconds) {
  //   return Commands.run(
  //       () -> {
  //         if (!intake.isTriggered()) {
  //           intake.runPercent(0.5);
  //         } else {
  //           new WaitCommand(seconds).andThen(intake.runPercent(0.0));
  //         }
  //       },
  //       intake);
  // }
}
