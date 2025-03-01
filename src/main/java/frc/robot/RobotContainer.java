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
// import com.pathplanner.lib.events.EventTrigger;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.commands.DriveCommands;
// import frc.robot.commands.IntakeCommands;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.led;
// import frc.robot.subsystems.climb.Climb;
// import frc.robot.subsystems.climb.ClimbConstants;
// import frc.robot.subsystems.climb.ClimbIO;
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
// import frc.robot.subsystems.elevator.ElevatorIO;
// import frc.robot.subsystems.elevator.ElevatorIOSim;
// import frc.robot.subsystems.elevator.ElevatorIOSpark;
// import frc.robot.subsystems.hopper.Hopper;
// import frc.robot.subsystems.hopper.HopperConstants;
// import frc.robot.subsystems.hopper.HopperIO;
// import frc.robot.subsystems.hopper.HopperIOSim;
// import frc.robot.subsystems.hopper.HopperIOSpark;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.IntakeConstants;
// import frc.robot.subsystems.intake.IntakeIO;
// import frc.robot.subsystems.intake.IntakeIOSim;
// import frc.robot.subsystems.intake.IntakeIOSpark;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.vision.VisionConstants;
// import frc.robot.subsystems.vision.VisionIO;
// import frc.robot.subsystems.vision.VisionIOLimelight;
// import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
// import frc.robot.subsystems.wrist.Wrist;
// import frc.robot.subsystems.wrist.WristConstants;
// import frc.robot.subsystems.wrist.WristIO;
// import frc.robot.subsystems.wrist.WristIOSim;
// import frc.robot.subsystems.wrist.WristIOSpark;
// import frc.robot.util.GlobalConstants;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link
// Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot
// (including
//  * subsystems, commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {
//   // Subsystems
//   private final Drive drive;
//   private final Elevator elevator;
//   private final Intake intake;
//   private final Wrist wrist;
//   private final Climb climber;
//   private final Hopper hopper;
//   private final Vision vision;
//   private final led led;

//   // Controller
//   private final CommandPS5Controller driverController = new CommandPS5Controller(0);
//   public final CommandPS5Controller operatorController = new CommandPS5Controller(1);

//   // Dashboard inputs
//   private final LoggedDashboardChooser<Command> autoChooser;

//   private final boolean startInManualMode = false;
//   private final boolean isInSoloDrivingMode = false;

//   private void configureInitialControllerBindings() {
//     drive.setDefaultCommand(
//         DriveCommands.joystickDrive(
//             drive,
//             () -> driverController.getLeftY(),
//             () -> driverController.getLeftX(),
//             () -> -driverController.getRightX()));
//     configureDriverControllerBindings();
//     if (startInManualMode) {
//       configureOperatorControllerManualModeBindings();
//     } else {
//       configureOperatorControllerSmartModeBindings();
//     }
//   }

//   private void configureDriverControllerBindings() {
//     driverController
//         .R3()
//         .whileTrue(
//             DriveCommands.joystickDriveAtAngle(
//                 drive,
//                 () -> drive.isRed() ? driverController.getLeftY() : -driverController.getLeftY(),
//                 () -> drive.isRed() ? driverController.getLeftX() : -driverController.getLeftX(),
//                 () -> Rotation2d.fromDegrees(drive.setAngle())));
//     driverController.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));
//     driverController
//         .PS()
//         .onTrue(
//             Commands.runOnce(
//                     () ->
//                         drive.setPose(
//                             new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
//                     drive)
//                 .ignoringDisable(true));
//   }

//   public void removeOperatorControllerBindings() {
//     CommandScheduler.getInstance().getActiveButtonLoop().clear();
//     configureDriverControllerBindings();
//   }

//   public void configureOperatorControllerManualModeBindings() {
//     if (isInSoloDrivingMode) {
//       driverController
//           .povUp()
//
// .whileTrue(elevator.runPercent(ElevatorConstants.speed).until(elevator::isAtUpperLimit));
//       driverController
//           .povDown()
//
// .whileTrue(elevator.runPercent(-ElevatorConstants.speed).until(elevator::isAtLowerLimit));
//       driverController.povLeft().whileTrue(climber.runPercent(ClimbConstants.speed));
//       driverController.povRight().whileTrue(climber.runPercent(-ClimbConstants.speed));
//       driverController.create().whileTrue(hopper.runPercent(HopperConstants.speed));
//       driverController.options().whileTrue(hopper.runPercent(-HopperConstants.speed));

//       driverController.L1().whileTrue(wrist.runPercent(WristConstants.speed));
//       driverController.R1().whileTrue(wrist.runPercent(-WristConstants.speed));

//       driverController.L2().whileTrue(intake.runPercent(-IntakeConstants.speed));
//       driverController.R2().whileTrue(intake.runPercent(IntakeConstants.speed));
//     } else {
//       operatorController
//           .povUp()
//
// .whileTrue(elevator.runPercent(ElevatorConstants.speed).until(elevator::isAtUpperLimit));
//       operatorController
//           .povDown()
//
// .whileTrue(elevator.runPercent(-ElevatorConstants.speed).until(elevator::isAtLowerLimit));
//       operatorController.povLeft().whileTrue(climber.runPercent(ClimbConstants.speed));
//       operatorController.povRight().whileTrue(climber.runPercent(-ClimbConstants.speed));

//       //   controller.triangle().whileTrue(null);
//       //   controller.cross().whileTrue(null);
//       //   controller.square().whileTrue(null);
//       //   controller.circle().whileTrue(null);

//       operatorController.create().whileTrue(hopper.runPercent(HopperConstants.speed));
//       operatorController.options().whileTrue(hopper.runPercent(-HopperConstants.speed));

//       operatorController.L1().whileTrue(wrist.runPercent(WristConstants.speed));
//       operatorController.R1().whileTrue(wrist.runPercent(-WristConstants.speed));

//       operatorController.L2().whileTrue(intake.runPercent(-IntakeConstants.speed));
//       operatorController.R2().whileTrue(intake.runPercent(IntakeConstants.speed));
//     }
//   }

//   public void configureOperatorControllerSmartModeBindings() {
//     if (isInSoloDrivingMode) {
//       driverController
//           .L2()
//           .whileTrue(
//               AutomatedCommands.homeCommand(wrist, elevator, led)
//                   .alongWith(
//                       IntakeCommands.runIntakeForward(intake)
//                           .alongWith(led.makeWholeColorCommand(Color.kRed))
//                           .until(intake::isTriggered))
//                   .andThen(led.makeWholeColorCommand(Color.kGreen)));

//       driverController.R2().whileTrue(IntakeCommands.runIntakeForward(intake));

//       driverController.options().whileTrue(climber.runPercent(-ClimbConstants.speed));
//       driverController.create().whileTrue(climber.runPercent(ClimbConstants.speed));

//       driverController
//           .triangle()
//           .whileTrue(AutomatedCommands.coralL4Command(intake, wrist, elevator, led));
//       driverController
//           .circle()
//           .whileTrue(AutomatedCommands.coralL3Command(intake, wrist, elevator, led));
//       driverController
//           .square()
//           .whileTrue(AutomatedCommands.coralL2Command(intake, wrist, elevator, led));

//       driverController.cross().whileTrue(IntakeCommands.runIntakeBackward(intake));

//       driverController
//           .povUp()
//           .whileTrue(
//               AutomatedCommands.netCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       driverController
//           .povDown()
//           .whileTrue(
//               AutomatedCommands.processorCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       driverController
//           .povLeft()
//           .whileTrue(
//               AutomatedCommands.algaeL2Command(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       driverController
//           .povRight()
//           .whileTrue(
//               AutomatedCommands.algaeL3Command(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       driverController
//           .touchpad()
//           .whileTrue(
//               AutomatedCommands.homeWithAlgaeCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));

//       driverController.R1().whileTrue(hopper.runPercent(-HopperConstants.speed));
//       driverController.L1().whileTrue(hopper.runPercent(HopperConstants.speed));
//     } else {
//       operatorController
//           .L2()
//           .whileTrue(
//               AutomatedCommands.homeCommand(wrist, elevator, led)
//                   .alongWith(
//                       IntakeCommands.runIntakeForward(intake)
//                           .alongWith(led.makeWholeColorCommand(Color.kRed))
//                           .until(intake::isTriggered))
//                   .andThen(led.makeWholeColorCommand(Color.kGreen)));

//       operatorController.R2().whileTrue(IntakeCommands.runIntakeForward(intake));

//       operatorController.options().whileTrue(climber.runPercent(-ClimbConstants.speed));
//       operatorController.create().whileTrue(climber.runPercent(ClimbConstants.speed));

//       operatorController
//           .triangle()
//           .whileTrue(AutomatedCommands.coralL4Command(intake, wrist, elevator, led));
//       operatorController
//           .circle()
//           .whileTrue(AutomatedCommands.coralL3Command(intake, wrist, elevator, led));
//       operatorController
//           .square()
//           .whileTrue(AutomatedCommands.coralL2Command(intake, wrist, elevator, led));

//       operatorController.cross().whileTrue(IntakeCommands.runIntakeBackward(intake));

//       operatorController
//           .povUp()
//           .whileTrue(
//               AutomatedCommands.netCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       operatorController
//           .povDown()
//           .whileTrue(
//               AutomatedCommands.processorCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       operatorController
//           .povLeft()
//           .whileTrue(
//               AutomatedCommands.algaeL2Command(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       operatorController
//           .povRight()
//           .whileTrue(
//               AutomatedCommands.algaeL3Command(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));
//       operatorController
//           .touchpad()
//           .whileTrue(
//               AutomatedCommands.homeWithAlgaeCommand(intake, wrist, elevator, led)
//                   .alongWith(IntakeCommands.runIntakeBackward(intake)));

//       operatorController.R1().whileTrue(hopper.runPercent(-HopperConstants.speed));
//       operatorController.L1().whileTrue(hopper.runPercent(HopperConstants.speed));
//     }
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
//         // Real robot, instantiate hardware IO implementations
//         drive =
//             new Drive(
//                 new GyroIOPigeon2(),
//                 new ModuleIOTalonFX(TunerConstants.FrontLeft),
//                 new ModuleIOTalonFX(TunerConstants.FrontRight),
//                 new ModuleIOTalonFX(TunerConstants.BackLeft),
//                 new ModuleIOTalonFX(TunerConstants.BackRight));
//         elevator = new Elevator(new ElevatorIOSpark());
//         intake = new Intake(new IntakeIOSpark());
//         wrist = new Wrist(new WristIOSpark());
//         climb = new Climb(new ClimbIOSpark());
//         hopper = new Hopper(new HopperIOSpark());
//         vision =
//             new Vision(
//                 drive::addVisionMeasurement,
//                 new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
//                 new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
//         led = new led();
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
//         intake = new Intake(new IntakeIOSim());
//         wrist = new Wrist(new WristIOSim());
//         climber = new Climb(new ClimbIOSim());
//         hopper = new Hopper(new HopperIOSim());
//         vision =
//             new Vision(
//                 drive::addVisionMeasurement,
//                 new VisionIOPhotonVisionSim(
//                     VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
//                 new VisionIOPhotonVisionSim(
//                     VisionConstants.camera1Name, VisionConstants.robotToCamera1,
// drive::getPose));
//         led = new led();
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
//         elevator =
//             new Elevator(
//                 new ElevatorIO() {

//                   @Override
//                   public Object getDistance() {
//                     // TODO Auto-generated method stub
//                     throw new UnsupportedOperationException("Unimplemented method
// 'getDistance'");
//                   }
//                 });
//         intake = new Intake(new IntakeIO() {});
//         wrist = new Wrist(new WristIO() {});
//         climber = new Climb(new ClimbIO() {});
//         hopper = new Hopper(new HopperIO() {});
//         vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
//         led = new led();
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

//     // Named Commands
//     NamedCommands.registerCommand("ScoreCoral", IntakeCommands.runIntakeForward(intake));
//     NamedCommands.registerCommand("IntakeCoral", AutomatedCommands.intakeCoral(intake,
// led));
//     NamedCommands.registerCommand(
//         "HomePos", AutomatedCommands.homeCommand(wrist, elevator, led));
//     NamedCommands.registerCommand(
//         "CoralL4Pos", AutomatedCommands.coralL4Command(intake, wrist, elevator, led));

//     new EventTrigger("l4 position").whileTrue(Commands.print("Going to L4 position"));
//     new EventTrigger("score coral").whileTrue(Commands.print("Score Coral"));
//     new EventTrigger("home position").whileTrue(Commands.print("Go Home"));
//     new EventTrigger("intake coral").whileTrue(Commands.print("intaking coral"));

//     // Configure the button bindings
//     configureInitialControllerBindings();
//     SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, false);
//     // configureButtonBindings();
//     updateDashboard();
//   }

//   public void updateDashboard() {
//     // Basic Bitch Commands
//     SmartDashboard.putData("Intake", intake.runPercent(0.5));
//     SmartDashboard.putData("Eject", intake.runPercent(-0.5));
//     SmartDashboard.putData("WristUp", wrist.runPercent(-0.1));
//     SmartDashboard.putData("WristDown", wrist.runPercent(0.1));
//     SmartDashboard.putData("ElevateUp",
// elevator.runPercent(0.1).until(elevator::isAtUpperLimit));
//     SmartDashboard.putData(
//         "ElevateDown", elevator.runPercent(-0.1).until(elevator::isAtLowerLimit));
//     SmartDashboard.putData("ExtendClimber", climber.runPercent(0.5));
//     SmartDashboard.putData("RetractClimber", climber.runPercent(-0.5));
//     SmartDashboard.putData("HopperUp", hopper.runPercent(0.1));
//     SmartDashboard.putData("HopperDown", hopper.runPercent(-0.1));

//     // Advanced Commands
//     SmartDashboard.putData("IntakeCoral", intake.runPercent(0.5).until(intake::isTriggered));
//     SmartDashboard.putData(
//         "WristHomePos", wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L1_POS));
//     SmartDashboard.putData(
//         "WristL23Pos", wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L3_POS));
//     SmartDashboard.putData(
//         "WristL4Pos", wrist.setWristPosition(WristConstants.WRIST_SCORE_CORAL_L4_POS));
//     SmartDashboard.putData(
//         "WristAlgaePos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_PICKUP_FLOOR_POS));
//     SmartDashboard.putData(
//         "WristProcPos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_SCORE_PROCESSOR_POS));
//     SmartDashboard.putData(
//         "WristNetPos", wrist.setWristPosition(WristConstants.WRIST_ALGAE_SCORE_NET_POS));
//     SmartDashboard.putData(
//         "ElevatorHomePos",
// elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_HOME_POSITION));
//     SmartDashboard.putData(
//         "ElevatorCoralL2Pos",
//         elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L2_POS));
//     SmartDashboard.putData(
//         "ElevatorCoralL3Pos",
//         elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L3_POS));
//     SmartDashboard.putData(
//         "ElevatorCoralL4Pos",
//         elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SCORE_CORAL_L4_POS));
//     SmartDashboard.putData(
//         "ElevatorAlgaeL2Pos",
//         elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L2_POS));
//     SmartDashboard.putData(
//         "ElevatorAlgaeL3Pos",
//         elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_ALGAE_PICKUP_L3_POS));
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands.GoScoreCoralL4;
import frc.robot.commands.AutomatedCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
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
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
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
  private final Intake intake;
  private final Wrist wrist;
  private final Hopper hopper;
  private final Climb climb;
  private final LEDStrip led;
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
    operatorController.L1().whileTrue(wrist.runPercent(0.5));
    operatorController.R1().whileTrue(wrist.runPercent(-0.5));
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
    operatorController
        .L2()
        .whileTrue(
            AutomatedCommands.homeCommand(wrist, elevator, led)
                .alongWith(
                    IntakeCommands.runIntakeForward(intake)
                        .alongWith(led.makeWholeColorCommand(Color.kRed))
                        .until(intake::isTriggered))
                .andThen(led.makeWholeColorCommand(Color.kGreen)));

    operatorController.R2().whileTrue(IntakeCommands.runIntakeForward(intake));

    operatorController.options().whileTrue(climb.runPercent(-ClimbConstants.speed));
    operatorController.create().whileTrue(climb.runPercent(ClimbConstants.speed));

    operatorController.triangle().whileTrue(new GoScoreCoralL4(intake, wrist, elevator, led));
    operatorController
        .circle()
        .whileTrue(AutomatedCommands.coralL3Command(intake, wrist, elevator, led));
    operatorController
        .square()
        .whileTrue(AutomatedCommands.coralL2Command(intake, wrist, elevator, led));

    operatorController.cross().whileTrue(IntakeCommands.runIntakeBackward(intake));

    operatorController
        .povUp()
        .whileTrue(
            AutomatedCommands.netCommand(intake, wrist, elevator, led)
                .alongWith(IntakeCommands.runIntakeBackward(intake)));
    operatorController
        .povDown()
        .whileTrue(
            AutomatedCommands.processorCommand(intake, wrist, elevator, led)
                .alongWith(IntakeCommands.runIntakeBackward(intake)));
    operatorController
        .povLeft()
        .whileTrue(
            AutomatedCommands.algaeL2Command(intake, wrist, elevator, led)
                .alongWith(IntakeCommands.runIntakeBackward(intake)));
    operatorController
        .povRight()
        .whileTrue(
            AutomatedCommands.algaeL3Command(intake, wrist, elevator, led)
                .alongWith(IntakeCommands.runIntakeBackward(intake)));
    operatorController
        .touchpad()
        .whileTrue(
            AutomatedCommands.homeWithAlgaeCommand(intake, wrist, elevator, led)
                .alongWith(IntakeCommands.runIntakeBackward(intake)));

    operatorController.R1().whileTrue(hopper.runPercent(-HopperConstants.speed));
    operatorController.L1().whileTrue(hopper.runPercent(HopperConstants.speed));
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
        led = new LEDStrip();

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
        hopper = new Hopper(new HopperIOSim());
        led = new LEDStrip();
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
            .onlyWhile(() -> !elevator.isAtUpperLimit()));
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
    SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, false);
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
    //     elevator.runTeleop(() -> driveController.getR2Axis(), () ->
    // driveController.getL2Axis()));
