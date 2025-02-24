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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands.GoHome;
import frc.robot.commands.AutoCommands.GoRemoveAlgaeL2;
import frc.robot.commands.AutoCommands.GoRemoveAlgaeL3;
import frc.robot.commands.AutoCommands.GoScoreAlgaeNet;
import frc.robot.commands.AutoCommands.GoScoreAlgaeProcessor;
import frc.robot.commands.AutoCommands.GoScoreCoralL1;
import frc.robot.commands.AutoCommands.GoScoreCoralL2;
import frc.robot.commands.AutoCommands.GoScoreCoralL3;
import frc.robot.commands.AutoCommands.GoScoreCoralL4;
import frc.robot.commands.AutoCommands.RunningIntakeBackwards;
import frc.robot.commands.AutoCommands.RunningIntakeForward;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDStrip;
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
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.util.GlobalConstants;
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
  private final Hopper hopper;
  private final Climb climb;
  private LEDStrip led;
  // Controller
  private final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final boolean startInManualMode = false;

  private void configureInitialControllerBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
    configureDriverControllerBindings();
    //////////
    if (startInManualMode) {
      configureOperatorControllerManualModeBindings();
    } else {
      configureOperatorControllerSmartModeBindings();
    }
  }

  private void configureDriverControllerBindings() {
    driveController
        .L3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));
    driveController.square().onTrue(Commands.runOnce(drive::stopWithX, drive));
     // Reset gyro to 0° when B button is pressed
     driveController
     .options()
     .onTrue(
         Commands.runOnce(
             () ->
               drive.setPose(
                 new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                   drive)
         .ignoringDisable(true));
  }

  public void removeOperatorControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings();
  }
  public void configureOperatorControllerManualModeBindings() {
  // elevetor
    operatorController.povUp().whileTrue(elevator.runPercent(0.5));
    operatorController.povDown().whileTrue(elevator.runPercent(-0.5));
  // wrist 
    operatorController.L1().whileTrue(intake.runPercent(0.5));
    operatorController.R1().whileTrue(intake.runPercent(-0.5));
  // hopper
    operatorController.R1().whileTrue(hopper.runPercent(0.5));
    operatorController.L1().whileTrue(hopper.runPercent(-0.5));
  // intake 
    operatorController.triangle().whileTrue(hopper.runPercent(0.5));
    operatorController.cross().whileTrue(hopper.runPercent(-0.5));
  // climb
    operatorController.options().whileTrue(climb.runPercent(0.5));
    operatorController.create().whileTrue(climb.runPercent(-0.5));
  }

  public void configureOperatorControllerSmartModeBindings() {
  // Running end effector operator controller
     operatorController.R2().whileTrue(new RunningIntakeForward(intake));
     operatorController.L2().whileTrue(new GoHome(wrist, elevator, intake));
  // scoring on the reef operator controller
     operatorController.triangle().whileTrue(new GoScoreCoralL4(intake, wrist, elevator, led));
     operatorController.circle().whileTrue(new GoScoreCoralL3(intake, wrist, elevator, led));
     operatorController.square().whileTrue(new GoScoreCoralL2(intake, wrist, elevator));
     operatorController.cross().whileTrue(new GoScoreCoralL1(wrist, elevator, intake, led));
  // algae scoring operator controller
     operatorController.povUp().whileTrue(new GoScoreAlgaeNet(intake, wrist, elevator));
     operatorController.povDown().whileTrue(new GoScoreAlgaeProcessor(intake, wrist, elevator));
     operatorController.povRight().whileTrue(new GoRemoveAlgaeL3(intake, wrist, elevator));
     operatorController.povLeft().whileTrue(new GoRemoveAlgaeL2(intake, wrist, elevator));
  // climb operator controller
     operatorController.L1().whileTrue(climb.runPercent(0.5));
     operatorController.R1().whileTrue(climb.runPercent(-0.5));
  }

  public void toggleManualModeWhenButtonPressed() {
    if (operatorController.getHID().getRawButtonPressed(15)) {
      boolean before = GlobalConstants.isManualMode();
      boolean after = !before;
      System.out.println("TOGGLE MANUAL MODE from " + before + " to " + after + ".");
      removeOperatorControllerBindings();
      SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, after);
      if (after) {
        configureOperatorControllerManualModeBindings();
      } else {
        configureOperatorControllerSmartModeBindings();
      }
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
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
        hopper = new Hopper(new HopperIOSpark());
        led = new LEDStrip();
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
        hopper = new Hopper(new HopperIOSim());
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
        led = new LEDStrip();
        hopper = new Hopper(new HopperIOSim());
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

    NamedCommands.registerCommand(
      "GoL4",
      wrist
          .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L4_POS)
          .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS))
          .onlyWhile(() -> !elevator.isAtUpperLimit())
    );
    NamedCommands.registerCommand(
        "GoL3",
        wrist
            .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS)
            .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L3_POS))
            .onlyWhile(() -> !elevator.isAtUpperLimit()));

    NamedCommands.registerCommand(
        "GoL2",
        wrist
            .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L2_POS)
            .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L2_POS))
            .onlyWhile(() -> !elevator.isAtUpperLimit()));
    NamedCommands.registerCommand(
        "GoL1",
        wrist
            .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L1_POS)
            .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L1_POS))
            .onlyWhile(() -> !elevator.isAtUpperLimit()));
    
    NamedCommands.registerCommand(
        "GoHome",
        wrist
            .setWristPosition(WristConstants.WRIST_HOME_POSITION)
            .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION))
            .onlyWhile(() -> !elevator.isAtLowerLimit()));
    

    NamedCommands.registerCommand("Eject", intake.runPercent(0.5));

    // Configure the button bindings
    configureInitialControllerBindings();
    smartDashBoardButtons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void smartDashBoardButtons() {
    SmartDashboard.putData("RunIntake In", intake.runPercent(5));
    SmartDashboard.putData("RunIntake Out", intake.runPercent(-5));

    SmartDashboard.putData("Wrist Up", wrist.runPercent(5));
    SmartDashboard.putData("Wrist Down", wrist.runPercent(-5));

    SmartDashboard.putData("Elevator Up", elevator.runPercent(5));
    SmartDashboard.putData("Elevator Down", elevator.runPercent(-5));
    SmartDashboard.putData("RunIntake", intake.runPercent(5));
    SmartDashboard.putData("RunIntake", intake.runPercent(5));
    SmartDashboard.putData("RunIntake", intake.runPercent(5));
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


    // // Default command, normal field-relative drive
    
    // elevator.setDefaultCommand(
    //     elevator.runTeleop(() -> driveController.getR2Axis(), () -> driveController.getL2Axis()));