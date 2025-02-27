package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutomatedCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffecterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
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
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOSpark;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
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
  private final EndEffector endEffecter;
  private final Wrist wrist;
  private final Climb climber;
  private final Hopper hopper;
  private final Vision vision;
  private final LEDStrip ledStrip;

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public final CommandPS5Controller operaController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final boolean startInManualMode = false;
  private final boolean isInSoloDrivingMode = false;

  private void configureInitialControllerBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX()));
    configureDriverControllerBindings();
    if (startInManualMode) {
      configureOperatorControllerManualModeBindings();
    } else {
      configureOperatorControllerSmartModeBindings();
    }
  }

  private void configureDriverControllerBindings() {
    driverController.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .PS()
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
    if (isInSoloDrivingMode) {
        driverController
        .povUp()
        .whileTrue(elevator.runPercent(ElevatorConstants.manualSpeed).until(elevator::isAtUpperLimit));
    driverController
        .povDown()
        .whileTrue(elevator.runPercent(-ElevatorConstants.manualSpeed).until(elevator::isAtLowerLimit));
        driverController.povLeft().whileTrue(climber.runPercent(ClimbConstants.manualSpeed));
        driverController.povRight().whileTrue(climber.runPercent(-ClimbConstants.manualSpeed));
        driverController.create().whileTrue(hopper.runPercent(HopperConstants.manualSpeed));
        driverController.options().whileTrue(hopper.runPercent(-HopperConstants.manualSpeed));

        driverController.L1().whileTrue(wrist.runPercent(WristConstants.manualSpeed));
        driverController.R1().whileTrue(wrist.runPercent(-WristConstants.manualSpeed));

        driverController.L2().whileTrue(endEffecter.runPercent(-EndEffectorConstants.speed));
        driverController.R2().whileTrue(endEffecter.runPercent(EndEffectorConstants.speed));
    }else{
    operaController
        .povUp()
        .whileTrue(elevator.runPercent(ElevatorConstants.manualSpeed).until(elevator::isAtUpperLimit));
    operaController
        .povDown()
        .whileTrue(elevator.runPercent(-ElevatorConstants.manualSpeed).until(elevator::isAtLowerLimit));
    operaController.povLeft().whileTrue(climber.runPercent(ClimbConstants.manualSpeed));
    operaController.povRight().whileTrue(climber.runPercent(-ClimbConstants.manualSpeed));

    operaController.create().whileTrue(hopper.runPercent(HopperConstants.manualSpeed));
    operaController.options().whileTrue(hopper.runPercent(-HopperConstants.manualSpeed));

    operaController.L1().whileTrue(wrist.runPercent(WristConstants.manualSpeed));
    operaController.R1().whileTrue(wrist.runPercent(-WristConstants.manualSpeed));

    operaController.L2().whileTrue(endEffecter.runPercent(-EndEffectorConstants.speed));
    operaController.R2().whileTrue(endEffecter.runPercent(EndEffectorConstants.speed));
    }
  }

  public void configureOperatorControllerSmartModeBindings() {
    if (isInSoloDrivingMode) {
        driverController
        .L2()
        .whileTrue(
            AutomatedCommands.homeCommand(wrist, elevator, ledStrip)
                .alongWith(
                    EndEffecterCommands.runEndEffecterForward(endEffecter)
                        .alongWith(ledStrip.makeWholeColorCommand(Color.kRed))
                        .until(endEffecter::isTriggered))
                .andThen(ledStrip.makeWholeColorCommand(Color.kGreen)));

                driverController.R2().whileTrue(EndEffecterCommands.runEndEffecterForward(endEffecter));

                driverController.options().whileTrue(climber.runPercent(-ClimbConstants.autoSpeed));
                driverController.create().whileTrue(climber.runPercent(ClimbConstants.autoSpeed));

                driverController
        .triangle()
        .whileTrue(AutomatedCommands.coralL4Command(endEffecter, wrist, elevator, ledStrip));
        driverController
        .circle()
        .whileTrue(AutomatedCommands.coralL3Command(endEffecter, wrist, elevator, ledStrip));
        driverController
        .square()
        .whileTrue(AutomatedCommands.coralL2Command(endEffecter, wrist, elevator, ledStrip));

        driverController.cross().whileTrue(EndEffecterCommands.runEndEffecterBackward(endEffecter));

        driverController
        .povUp()
        .whileTrue(
            AutomatedCommands.netCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
                driverController
        .povDown()
        .whileTrue(
            AutomatedCommands.processorCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
                driverController
        .povLeft()
        .whileTrue(
            AutomatedCommands.algaeL2Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
                driverController
        .povRight()
        .whileTrue(
            AutomatedCommands.algaeL3Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
                driverController
        .touchpad()
        .whileTrue(
            AutomatedCommands.homeWithAlgaeCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));

                driverController.R1().whileTrue(hopper.runPercent(-HopperConstants.autoSpeed));
                driverController.L1().whileTrue(hopper.runPercent(HopperConstants.autoSpeed));
    }else{
    operaController
        .L2()
        .whileTrue(
            AutomatedCommands.homeCommand(wrist, elevator, ledStrip)
                .alongWith(
                    EndEffecterCommands.runEndEffecterForward(endEffecter)
                        .alongWith(ledStrip.makeWholeColorCommand(Color.kRed))
                        .until(endEffecter::isTriggered))
                .andThen(ledStrip.makeWholeColorCommand(Color.kGreen)));

    operaController.R2().whileTrue(EndEffecterCommands.runEndEffecterForward(endEffecter));

    operaController.options().whileTrue(climber.runPercent(-ClimbConstants.autoSpeed));
    operaController.create().whileTrue(climber.runPercent(ClimbConstants.autoSpeed));

    operaController
        .triangle()
        .whileTrue(AutomatedCommands.coralL4Command(endEffecter, wrist, elevator, ledStrip));
    operaController
        .circle()
        .whileTrue(AutomatedCommands.coralL3Command(endEffecter, wrist, elevator, ledStrip));
    operaController
        .square()
        .whileTrue(AutomatedCommands.coralL2Command(endEffecter, wrist, elevator, ledStrip));

    operaController.cross().whileTrue(EndEffecterCommands.runEndEffecterBackward(endEffecter));

    operaController
        .povUp()
        .whileTrue(
            AutomatedCommands.netCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    operaController
        .povDown()
        .whileTrue(
            AutomatedCommands.processorCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    operaController
        .povLeft()
        .whileTrue(
            AutomatedCommands.algaeL2Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    operaController
        .povRight()
        .whileTrue(
            AutomatedCommands.algaeL3Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    operaController
        .touchpad()
        .whileTrue(
            AutomatedCommands.homeWithAlgaeCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));

    operaController.R1().whileTrue(hopper.runPercent(-HopperConstants.autoSpeed));
    operaController.L1().whileTrue(hopper.runPercent(HopperConstants.autoSpeed));
    }
  }

  public void toggleManualModeWhenButtonPressed() {
    if (operaController.getHID().getRawButtonPressed(15)) {
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
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSpark());
        endEffecter = new EndEffector(new EndEffectorIOSpark());
        wrist = new Wrist(new WristIOSpark());
        climber = new Climb(new ClimbIOSpark());
        hopper = new Hopper(new HopperIOSpark());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        ledStrip = new LEDStrip();
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
        endEffecter = new EndEffector(new EndEffectorIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climb(new ClimbIOSim());
        hopper = new Hopper(new HopperIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        ledStrip = new LEDStrip();
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
        elevator = new Elevator(new ElevatorIO() {});
        endEffecter = new EndEffector(new EndEffectorIO() {});
        wrist = new Wrist(new WristIO() {});
        climber = new Climb(new ClimbIO() {});
        hopper = new Hopper(new HopperIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        ledStrip = new LEDStrip();
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

    // Named Commands
    NamedCommands.registerCommand(
        "ScoreCoral", EndEffecterCommands.runEndEffecterForward(endEffecter));
    NamedCommands.registerCommand(
        "IntakeCoral", AutomatedCommands.intakeCoral(endEffecter, ledStrip));
    NamedCommands.registerCommand(
        "HomePos", AutomatedCommands.homeCommand(wrist, elevator, ledStrip));
    NamedCommands.registerCommand(
        "CoralL4Pos", AutomatedCommands.coralL4Command(endEffecter, wrist, elevator, ledStrip));

    new EventTrigger("l4 position").whileTrue(Commands.print("Going to L4 position"));
    new EventTrigger("score coral").whileTrue(Commands.print("Score Coral"));
    new EventTrigger("home position").whileTrue(Commands.print("Go Home"));
    new EventTrigger("intake coral").whileTrue(Commands.print("intaking coral"));

    // Configure the button bindings
    configureInitialControllerBindings();
    SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, false);
    // configureButtonBindings();
    updateDashboard();
  }

  public void updateDashboard() {
    // Basic Bitch Commands
    SmartDashboard.putData("Intake", endEffecter.runPercent(0.5));
    SmartDashboard.putData("Eject", endEffecter.runPercent(-0.5));
    SmartDashboard.putData("WristUp", wrist.runPercent(-0.1));
    SmartDashboard.putData("WristDown", wrist.runPercent(0.1));
    SmartDashboard.putData("ElevateUp", elevator.runPercent(0.1).until(elevator::isAtUpperLimit));
    SmartDashboard.putData("ElevateDown", elevator.runPercent(-0.1).until(elevator::isAtLowerLimit));
    SmartDashboard.putData("ExtendClimber", climber.runPercent(0.5));
    SmartDashboard.putData("RetractClimber", climber.runPercent(-0.5));
    SmartDashboard.putData("HopperUp", hopper.runPercent(0.1));
    SmartDashboard.putData("HopperDown", hopper.runPercent(-0.1));

    // Advanced Commands
    SmartDashboard.putData(
        "IntakeCoral", endEffecter.runPercent(0.5).until(endEffecter::isTriggered));
    SmartDashboard.putData("WristHomePos", wrist.setWristPosition(WristConstants.WRIST_HOME_POSITION));
    SmartDashboard.putData("WristL23Pos", wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS));
    SmartDashboard.putData("WristL4Pos", wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L4_POS));
    SmartDashboard.putData("WristAlgaePos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_L3_POS));
    SmartDashboard.putData(
        "WristProcPos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_SCORE_PROCESSOR_POS));
    SmartDashboard.putData("WristNetPos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_SCORE_NET_POS));
    SmartDashboard.putData(
        "ElevatorHomePos", elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION));
    SmartDashboard.putData(
        "ElevatorCoralL2Pos",
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L2_POS));
    SmartDashboard.putData(
        "ElevatorCoralL3Pos",
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS));
    SmartDashboard.putData(
        "ElevatorCoralL4Pos",
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS));
    SmartDashboard.putData(
        "ElevatorAlgaeL2Pos",
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L2_POS));
    SmartDashboard.putData(
        "ElevatorAlgaeL3Pos",
        elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS));
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

// // Copyright 2021-2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.commands.DriveCommands;
// import frc.robot.commands.AutoComFolder.GoHome;
// import frc.robot.commands.AutoComFolder.GoRemoveAlgaeL2;
// import frc.robot.commands.AutoComFolder.GoRemoveAlgaeL3;
// import frc.robot.commands.AutoComFolder.GoScoreAlgaeNet;
// import frc.robot.commands.AutoComFolder.GoScoreAlgaeProcessor;
// import frc.robot.commands.AutoComFolder.GoScoreCoralL2;
// import frc.robot.commands.AutoComFolder.GoScoreCoralL3;
// import frc.robot.commands.AutoComFolder.GoScoreCoralL4;
// import frc.robot.commands.AutoComFolder.RunningIntakeBackwards;
// import frc.robot.commands.AutoComFolder.RunningIntakeForward;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.LEDStrip;
// import frc.robot.subsystems.climb.Climb;
// import frc.robot.subsystems.climb.ClimbIOSim;
// import frc.robot.subsystems.climb.ClimbIOSpark;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
// import frc.robot.subsystems.drive.ModuleIO;
// import frc.robot.subsystems.drive.ModuleIOSim;
// import frc.robot.subsystems.drive.ModuleIOTalonFX;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.ElevatorConstants;
// import frc.robot.subsystems.elevator.ElevatorIOSim;
// import frc.robot.subsystems.elevator.ElevatorIOSpark;
// import frc.robot.subsystems.endEffecter.EndEffector;
// import frc.robot.subsystems.endEffecter.EndEffecterIOSim;
// import frc.robot.subsystems.endEffecter.EndEffecterIOSpark;
// import frc.robot.subsystems.hopper.Hopper;
// import frc.robot.subsystems.hopper.HopperIOSim;
// import frc.robot.subsystems.hopper.HopperIOSpark;
// import frc.robot.subsystems.wrist.Wrist;
// import frc.robot.subsystems.wrist.WristConstants;
// import frc.robot.subsystems.wrist.WristIOSim;
// import frc.robot.subsystems.wrist.WristIOSpark;
// import frc.robot.util.GlobalConstants;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {
//   // Subsystems
//   private final Drive drive;
//   private final Elevator elevator;
//   private final EndEffector intake;
//   private final Wrist wrist;
//   private final Hopper hopper;
//   private final Climb climb;
//   private LEDStrip led;
//   // Controller
//   private final CommandPS5Controller driveController = new CommandPS5Controller(0);
//   private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

//   // Dashboard inputs
//   private final LoggedDashboardChooser<Command> autoChooser;
//   private final boolean startInManualMode = false;

//   private void configureInitialControllerBindings() {
//     drive.setDefaultCommand(
//         DriveCommands.joystickDrive(
//             drive,
//             () -> -driveController.getLeftY(),
//             () -> -driveController.getLeftX(),
//             () -> -driveController.getRightX()));
//     configureDriverControllerBindings();
//     //////////
//     if (startInManualMode) {
//       configureOperatorControllerManualModeBindings();
//     } else {
//       configureOperatorControllerSmartModeBindings();
//     }
//   }

//   private void configureDriverControllerBindings() {
//     driveController
//         .L3()
//         .whileTrue(
//             DriveCommands.joystickDriveAtAngle(
//                 drive,
//                 () -> -driveController.getLeftY(),
//                 () -> -driveController.getLeftX(),
//                 () -> new Rotation2d()));
//   }

//   public void removeOperatorControllerBindings() {
//     CommandScheduler.getInstance().getActiveButtonLoop().clear();
//     configureDriverControllerBindings();
//   }

//   public void configureOperatorControllerManualModeBindings() {
//     // intake
//     operatorController.L2().whileTrue(new RunningIntakeBackwards(intake));
//     operatorController.R2().whileTrue(new RunningIntakeForward(intake));
//     // coral
//     operatorController.triangle().whileTrue(new GoScoreCoralL4(intake, wrist, elevator, led));
//     operatorController.circle().whileTrue(new GoScoreCoralL3(intake, wrist, elevator, led));
//     operatorController.square().whileTrue(new GoScoreCoralL2(intake, wrist, elevator, led));

//     // algae
//     operatorController.povRight().whileTrue(new GoRemoveAlgaeL3(intake, wrist, elevator, led));
//     operatorController.povLeft().whileTrue(new GoRemoveAlgaeL2(intake, wrist, elevator, led));
//     operatorController.povUp().whileTrue(new GoScoreAlgaeNet(intake, wrist, elevator, led));
//     operatorController.povDown().whileTrue(new GoScoreAlgaeProcessor(intake, wrist, elevator, led));
//     // home
//     operatorController.L2().whileTrue(new GoHome(wrist, elevator, intake));
//   }

//   public void configureOperatorControllerSmartModeBindings() {
//     // wrist
//     operatorController.L2().whileTrue(wrist.runPercent(-0.4));
//     operatorController.R2().whileTrue(wrist.runPercent(0.4));
//     // elevator
//     operatorController.povUp().whileTrue(elevator.runPercent(0.4));
//     operatorController.povDown().whileTrue(elevator.runPercent(-0.4));
//     // intake
//     operatorController.R1().whileTrue(intake.runPercent(-0.4));
//     operatorController.L1().whileTrue(intake.runPercent(-0.4));
//     // ram
//     operatorController.triangle().whileTrue(hopper.runPercent(0.4));
//     operatorController.cross().whileTrue(hopper.runPercent(-0.4));
//     // climber
//     operatorController.circle().whileTrue(climb.runPercent(-0.4));
//     operatorController.R1().whileTrue(climb.runPercent(-0.4));
//   }

//   public void toggleManualModeWhenButtonPressed() {
//     if (operatorController.getHID().getRawButtonPressed(15)) {
//       boolean before = GlobalConstants.isManualMode();
//       boolean after = !before;
//       System.out.println("TOGGLE MANUAL MODE from " + before + " to " + after + ".");
//       removeOperatorControllerBindings();
//       SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, after);
//       if (after) {
//         configureOperatorControllerManualModeBindings();
//       } else {
//         configureOperatorControllerSmartModeBindings();
//       }
//     }
//   }

//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     switch (Constants.currentMode) {
//       case REAL:
//         drive =
//             new Drive(
//                 new GyroIOPigeon2(),
//                 new ModuleIOTalonFX(TunerConstants.FrontLeft),
//                 new ModuleIOTalonFX(TunerConstants.FrontRight),
//                 new ModuleIOTalonFX(TunerConstants.BackLeft),
//                 new ModuleIOTalonFX(TunerConstants.BackRight));
//         elevator = new Elevator(new ElevatorIOSpark());
//         intake = new EndEffector(new EndEffecterIOSpark());
//         wrist = new Wrist(new WristIOSpark());
//         climb = new Climb(new ClimbIOSpark());
//         hopper = new Hopper(new HopperIOSpark());
//         led = new LEDStrip();
//         break;

//       case SIM:
//         // Sim robot, instantiate physics sim IO implementations
//         drive =
//             new Drive(
//                 new GyroIO() {},
//                 new ModuleIOSim(TunerConstants.FrontLeft),
//                 new ModuleIOSim(TunerConstants.FrontRight),
//                 new ModuleIOSim(TunerConstants.BackLeft),
//                 new ModuleIOSim(TunerConstants.BackRight));
//         elevator = new Elevator(new ElevatorIOSim());
//         intake = new EndEffector(new EndEffecterIOSim());
//         wrist = new Wrist(new WristIOSim());
//         climb = new Climb(new ClimbIOSim());
//         hopper = new Hopper(new HopperIOSim());
//         // vision = new Vision(new VisionIOPhotonVisionSim(/*TODO: figure out the name of this */));

//         break;

//       default:
//         // Replayed robot, disable IO implementations
//         drive =
//             new Drive(
//                 new GyroIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {});
//         elevator = new Elevator(new ElevatorIOSim());
//         intake = new EndEffector(new EndEffecterIOSim());
//         wrist = new Wrist(new WristIOSim());
//         climb = new Climb(new ClimbIOSim());
//         led = new LEDStrip();
//         hopper = new Hopper(new HopperIOSim());
//         break;
//     }

//     // Set up auto routines
//     autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

//     // Set up SysId routines
//     autoChooser.addOption(
//         "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
//     autoChooser.addOption(
//         "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
//     autoChooser.addOption(
//         "Drive SysId (Quasistatic Forward)",
//         drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//     autoChooser.addOption(
//         "Drive SysId (Quasistatic Reverse)",
//         drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//     autoChooser.addOption(
//         "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
//     autoChooser.addOption(
//         "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//     // NamedCommands.registerCommand("Intake", getAutonomousCommand());

//     NamedCommands.registerCommand(
//         "GoL4",
//         wrist
//             .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L4_POS)
//             .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS))
//             .onlyWhile(() -> !elevator.isAtUpperLimit()));
//     NamedCommands.registerCommand(
//         "GoL3",
//         wrist
//             .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS)
//             .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L3_POS))
//             .onlyWhile(() -> !elevator.isAtUpperLimit()));

//     NamedCommands.registerCommand(
//         "GoL2",
//         wrist
//             .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L2_POS)
//             .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L2_POS))
//             .onlyWhile(() -> !elevator.isAtUpperLimit()));
//     NamedCommands.registerCommand(
//         "GoL1",
//         wrist
//             .setWristPosition(WristConstants.WRIST_SCORE_CORAL_L1_POS)
//             .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L1_POS))
//             .onlyWhile(() -> !elevator.isAtUpperLimit()));

//     NamedCommands.registerCommand(
//         "GoHome",
//         wrist
//             .setWristPosition(WristConstants.WRIST_HOME_POSITION)
//             .andThen(elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION))
//             .onlyWhile(() -> !elevator.isAtLowerLimit()));

//     NamedCommands.registerCommand("Eject", intake.runPercent(0.5));

//     // Configure the button bindings
//     configureInitialControllerBindings();
//     SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, false);
//     smartDashBoardButtons();
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void smartDashBoardButtons() {
//     SmartDashboard.putData("RunIntake In", intake.runPercent(5));
//     SmartDashboard.putData("RunIntake Out", intake.runPercent(-5));

//     SmartDashboard.putData("Wrist Up", wrist.runPercent(5));
//     SmartDashboard.putData("Wrist Down", wrist.runPercent(-5));

//     SmartDashboard.putData("Elevator Up", elevator.runPercent(5));
//     SmartDashboard.putData("Elevator Down", elevator.runPercent(-5));
//     SmartDashboard.putData("RunIntake", intake.runPercent(5));
//     SmartDashboard.putData("RunIntake", intake.runPercent(5));
//     SmartDashboard.putData("RunIntake", intake.runPercent(5));
//   }

//   private void configureButtonBindings() {
//     // Default command, normal field-relative drive
//     drive.setDefaultCommand(
//         DriveCommands.joystickDrive(
//             drive,
//             () -> -driveController.getLeftY(),
//             () -> -driveController.getLeftX(),
//             () -> -driveController.getRightX()));

//     elevator.setDefaultCommand(
//         elevator.runTeleop(() -> driveController.getR2Axis(), () -> driveController.getL2Axis()));

//     // Lock to 0° when A button is held
//     driveController
//         .L3()
//         .whileTrue(
//             DriveCommands.joystickDriveAtAngle(
//                 drive,
//                 () -> -driveController.getLeftY(),
//                 () -> -driveController.getLeftX(),
//                 () -> new Rotation2d()));

//     // Switch to X pattern when X button is pressed(locking wheels)
//     driveController.square().onTrue(Commands.runOnce(drive::stopWithX, drive));
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return autoChooser.get();
//   }
// }

/////////////////////////////////////////////////////////////////////////
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

