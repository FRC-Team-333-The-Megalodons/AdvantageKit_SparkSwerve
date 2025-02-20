// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoCommands.GoHome;
import frc.robot.commands.AutoCommands.GoRemoveAlgaeL2;
import frc.robot.commands.AutoCommands.GoRemoveAlgaeL3;
import frc.robot.commands.AutoCommands.GoScoreAlgaeNet;
import frc.robot.commands.AutoCommands.GoScoreAlgaeProcessor;
import frc.robot.commands.AutoCommands.GoScoreCoralL2;
import frc.robot.commands.AutoCommands.GoScoreCoralL3;
import frc.robot.commands.AutoCommands.GoScoreCoralL4;
import frc.robot.commands.AutoCommands.RunningClimberBackwards;
import frc.robot.commands.AutoCommands.RunningClimberForward;
import frc.robot.commands.AutoCommands.RunningIntakeBackwards;
import frc.robot.commands.AutoCommands.RunningIntakeForward;
import frc.robot.commands.AutoCommands.RunningRampDown;
import frc.robot.commands.AutoCommands.RunningRampUp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;
  private final Climb climb;
  //   private final Vision vision;

  // Controller
  private final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSpark());
        intake = new Intake(new IntakeIOSpark());
        wrist = new Wrist(new WristIOSpark());
        climb = new Climb(new ClimbIOSpark());
        // vision = new Vision(new VisionIOPhotonVision(/* TODO: figure out the name of this */));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSim());
        intake = new Intake(new IntakeIOSim());
        wrist = new Wrist(new WristIOSim());
        climb = new Climb(new ClimbIOSim());
        // vision = new Vision(new VisionIOPhotonVisionSim(/*TODO: figure out the name of this */));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIOSim());
        intake = new Intake(new IntakeIOSim());
        wrist = new Wrist(new WristIOSim());
        climb = new Climb(new ClimbIOSim());
        // vision = new Vision(new VisionIOPhotonVisionSim(/*TODO: figure out the name of this */));
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // NamedCommands.registerCommand("Intake", getAutonomousCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    elevator.setDefaultCommand(
        elevator.runTeleop(() -> driveController.getR2Axis(), () -> driveController.getL2Axis()));

    // Lock to 0° when A button is held
    driveController
        .L3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed(locking wheels)
    driveController.square().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Running elevator drive controller
    driveController
        .povUp()
        .whileTrue(
            elevator
                .runPercent(0.1)
                .until(elevator::isAtUpperLimit)); // .until(elevator::isTriggeredLowLimit));
    driveController
        .povDown()
        .whileTrue(
            elevator
                .runPercent(-0.1)
                .until(elevator::isAtLowerLimit)); // .until(elevator::isTriggeredTopLimit));


    // Running wrist drive controller
    driveController.povLeft().whileTrue(wrist.runPercent(-0.4));
    driveController.povRight().whileTrue(wrist.runPercent(0.4));

    // Running climber drive controller
    driveController.R1().whileTrue(climb.runPercent(1));
    driveController.L1().whileTrue(climb.runPercent(-1));



    // Running end effector operator controller
    operatorController.cross().whileTrue(new RunningIntakeBackwards(intake, null));
    operatorController.R2().whileTrue(new RunningIntakeForward(intake, null));

    // Scoring Reef operator controller
    // L4
    operatorController.triangle().whileTrue(new GoScoreCoralL4(intake, wrist, elevator, null));
    // L3
    operatorController.circle().whileTrue(new GoScoreCoralL3(intake, wrist, elevator, null));
    // L2
    operatorController.square().whileTrue(new GoScoreCoralL2(intake, wrist, elevator, null));

    // Scoring and removing algae operator controller
    // Removing L3
    operatorController.povRight().whileTrue(new GoRemoveAlgaeL3(intake, wrist, elevator, null));
    // Removing L2
    operatorController.povLeft().whileTrue(new GoRemoveAlgaeL2(intake, wrist, elevator, null));
    // Scoring to the NET
    operatorController.povUp().whileTrue(new GoScoreAlgaeNet(intake, wrist, elevator, null));
    // Scoring Processor
    operatorController.povDown().whileTrue(new GoScoreAlgaeProcessor(intake, wrist, elevator, null));
    
    // Home position operator controller
    operatorController.L2().whileTrue(new GoHome(wrist, elevator, intake, null));

    // Running climber operator controller
    operatorController.L1().whileTrue(new RunningClimberBackwards(climb, null));
    operatorController.R1().whileTrue(new RunningClimberForward(climb, null));

    //Running ramp operator controller
    operatorController.options().whileTrue(new RunningRampDown(null, null));
    operatorController.create().whileTrue(new RunningRampUp(null, null));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
