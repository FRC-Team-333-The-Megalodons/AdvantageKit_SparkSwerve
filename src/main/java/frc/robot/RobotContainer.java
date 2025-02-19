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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
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
import frc.robot.subsystems.endEffecter.EndEffecter;
import frc.robot.subsystems.endEffecter.EndEffecterIO;
import frc.robot.subsystems.endEffecter.EndEffecterIOSim;
import frc.robot.subsystems.endEffecter.EndEffecterIOSpark;
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
  private final EndEffecter endEffecter;
  private final Wrist wrist;
  private final Climber climber;
  private final Vision vision;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

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
        endEffecter = new EndEffecter(new EndEffecterIOSpark());
        wrist = new Wrist(new WristIOSpark());
        climber = new Climber(new ClimberIOSpark());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
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
        endEffecter = new EndEffecter(new EndEffecterIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climber(new ClimberIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
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
        endEffecter = new EndEffecter(new EndEffecterIO() {});
        wrist = new Wrist(new WristIO() {});
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

    // Configure the button bindings
    configureButtonBindings();
    updateDashboard();
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
            () -> drive.isRed() ? controller.getLeftY() : -controller.getLeftY(),
            () -> drive.isRed() ? controller.getLeftX() : -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when R3 button is held
    // controller
    //     .R3()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    controller
        .R3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> drive.isRed() ? controller.getLeftY() : -controller.getLeftY(),
                () -> drive.isRed() ? controller.getLeftX() : -controller.getLeftX(),
                () -> vision.getTargetX(0)));

    // controller.R3().whileTrue(endEffecter.runPercent(-0.5));

    // Switch to X pattern when L3 button is pressed
    controller.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reef scoring buttons
    // L1
    controller.cross().whileTrue(endEffecter.runPercent(0.5));
    // L3
    controller
        .circle()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.coralL23Setpoint)
                .withTimeout(1.0)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.coralL3Setpoint)));
    // L2
    controller
        .square()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.coralL23Setpoint)
                .withTimeout(1.0)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.coralL2Setpoint)));
    // L4
    controller
        .triangle()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.coralL4Setpoint)
                .until(wrist::atSetpoint)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint))
                .until(elevator::upperLimit));

    // Algae scoring buttons
    // Barge score
    controller
        .povUp()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.bargeSetPoint)
                .until(wrist::atSetpoint)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.bargeSetPoint))
                .until(elevator::upperLimit));
    // Processor score
    controller
        .povDown()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.processorSetpoint)
                .until(wrist::atSetpoint)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.processorSetpoint)));
    // Remove L3
    controller
        .povRight()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.aglaeSetpoint)
                .until(wrist::atSetpoint)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.aglaeL3Setpoint)));
    // Remove L2
    controller
        .povLeft()
        .whileTrue(
            wrist
                .setWristPosition(WristConstants.aglaeSetpoint)
                .until(wrist::atSetpoint)
                .andThen(elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint)));

    // Intake from the coral station
    controller.R2().whileTrue(endEffecter.runPercent(0.5).until(endEffecter::isTriggered));

    // Robot home position
    controller
        .L2()
        .whileTrue(
            elevator
                .setElevatorPosition(ElevatorConstants.homeSetpoint)
                .until(elevator::atSetpoint)
                .andThen(wrist.setWristPosition(WristConstants.homeSetpoint)));

    // Retract
    controller.R1().whileTrue(climber.runPercent(1.0));
    // Extract
    controller.L1().whileTrue(climber.runPercent(-1.0));

    // Reset gyro to 0° when options button is pressed
    controller
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public void updateDashboard() {
    // Basic Bitch Commands
    SmartDashboard.putData("Intake", endEffecter.runPercent(0.5));
    SmartDashboard.putData("Eject", endEffecter.runPercent(-0.5));
    SmartDashboard.putData("WristUp", wrist.runPercent(-0.1));
    SmartDashboard.putData("WristDown", wrist.runPercent(0.1));
    SmartDashboard.putData("ElevateUp", elevator.runPercent(0.1).until(elevator::upperLimit));
    SmartDashboard.putData("ElevateDown", elevator.runPercent(-0.1).until(elevator::lowerLimit));
    SmartDashboard.putData("ExtendClimber", climber.runPercent(0.5));
    SmartDashboard.putData("RetractClimber", climber.runPercent(-0.5));

    // Advanced Commands
    SmartDashboard.putData(
        "IntakeCoral", endEffecter.runPercent(0.5).until(endEffecter::isTriggered));
    SmartDashboard.putData("WristHomePos", wrist.setWristPosition(WristConstants.homeSetpoint));
    SmartDashboard.putData("WristL23Pos", wrist.setWristPosition(WristConstants.coralL23Setpoint));
    SmartDashboard.putData("WristL4Pos", wrist.setWristPosition(WristConstants.coralL4Setpoint));
    SmartDashboard.putData("WristAlgaePos", wrist.setWristPosition(WristConstants.aglaeSetpoint));
    SmartDashboard.putData(
        "ElevatorHomePos", elevator.setElevatorPosition(ElevatorConstants.homeSetpoint));
    SmartDashboard.putData(
        "ElevatorCoralL2Pos", elevator.setElevatorPosition(ElevatorConstants.coralL2Setpoint));
    SmartDashboard.putData(
        "ElevatorCoralL3Pos", elevator.setElevatorPosition(ElevatorConstants.coralL3Setpoint));
    SmartDashboard.putData(
        "ElevatorCoralL4Pos", elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint));
    SmartDashboard.putData(
        "ElevatorAlgaeL2Pos", elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint));
    SmartDashboard.putData(
        "ElevatorAlgaeL3Pos", elevator.setElevatorPosition(ElevatorConstants.aglaeL3Setpoint));
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
