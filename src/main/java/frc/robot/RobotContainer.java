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
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
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
import frc.robot.subsystems.endEffecter.EndEffecterConstants;
import frc.robot.subsystems.endEffecter.EndEffecterIO;
import frc.robot.subsystems.endEffecter.EndEffecterIOSim;
import frc.robot.subsystems.endEffecter.EndEffecterIOSpark;
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
  private final EndEffecter endEffecter;
  private final Wrist wrist;
  private final Climber climber;
  private final Hopper hopper;
  private final Vision vision;
  private final LEDStrip ledStrip;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final boolean startInManualMode = false;

  private void configureInitialControllerBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
    configureDriverControllerBindings();
    if (startInManualMode) {
      configureOperatorControllerManualModeBindings();
    } else {
      configureOperatorControllerSmartModeBindings();
    }
  }

  private void configureDriverControllerBindings() {
    controller
        .R3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> drive.isRed() ? controller.getLeftY() : -controller.getLeftY(),
                () -> drive.isRed() ? controller.getLeftX() : -controller.getLeftX(),
                () -> Rotation2d.fromDegrees(drive.setAngle())));
    controller.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
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
    controller
        .povUp()
        .whileTrue(elevator.runPercent(ElevatorConstants.speed).until(elevator::upperLimit));
    controller
        .povDown()
        .whileTrue(elevator.runPercent(-ElevatorConstants.speed).until(elevator::lowerLimit));
    controller.povLeft().whileTrue(climber.runPercent(ClimberConstants.speed));
    controller.povRight().whileTrue(climber.runPercent(-ClimberConstants.speed));

    //   controller.triangle().whileTrue(null);
    //   controller.cross().whileTrue(null);
    //   controller.square().whileTrue(null);
    //   controller.circle().whileTrue(null);

    controller.create().whileTrue(hopper.runPercent(HopperConstants.speed));
    controller.options().whileTrue(hopper.runPercent(-HopperConstants.speed));

    controller.L1().whileTrue(wrist.runPercent(WristConstants.speed));
    controller.R1().whileTrue(wrist.runPercent(-WristConstants.speed));

    controller.L2().whileTrue(endEffecter.runPercent(-EndEffecterConstants.speed));
    controller.R2().whileTrue(endEffecter.runPercent(EndEffecterConstants.speed));
  }

  public void configureOperatorControllerSmartModeBindings() {
    controller
        .L2()
        .whileTrue(
            AutomatedCommands.homeCommand(wrist, elevator, ledStrip)
                .alongWith(
                    EndEffecterCommands.runEndEffecterForward(endEffecter)
                        .alongWith(ledStrip.makeWholeColorCommand(Color.kRed))
                        .until(endEffecter::isTriggered))
                .andThen(ledStrip.makeWholeColorCommand(Color.kGreen)));

    controller.R2().whileTrue(EndEffecterCommands.runEndEffecterForward(endEffecter));

    controller.L1().whileTrue(climber.runPercent(-ClimberConstants.speed));
    controller.R1().whileTrue(climber.runPercent(ClimberConstants.speed));

    controller
        .triangle()
        .whileTrue(AutomatedCommands.coralL4Command(endEffecter, wrist, elevator, ledStrip));
    controller
        .circle()
        .whileTrue(AutomatedCommands.coralL3Command(endEffecter, wrist, elevator, ledStrip));
    controller
        .square()
        .whileTrue(AutomatedCommands.coralL2Command(endEffecter, wrist, elevator, ledStrip));

    controller.cross().whileTrue(EndEffecterCommands.runEndEffecterBackward(endEffecter));

    controller
        .povUp()
        .whileTrue(
            AutomatedCommands.netCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    controller
        .povDown()
        .whileTrue(
            AutomatedCommands.processorCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    controller
        .povLeft()
        .whileTrue(
            AutomatedCommands.algaeL2Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    controller
        .povRight()
        .whileTrue(
            AutomatedCommands.algaeL3Command(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
    controller
        .touchpad()
        .whileTrue(
            AutomatedCommands.homeWithAlgaeCommand(endEffecter, wrist, elevator, ledStrip)
                .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));

    controller.create().whileTrue(hopper.runPercent(-HopperConstants.speed));
    controller.options().whileTrue(hopper.runPercent(HopperConstants.speed));
  }

  public void toggleManualModeWhenButtonPressed() {
    if (controller.getHID().getRawButtonPressed(15)) {
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
        endEffecter = new EndEffecter(new EndEffecterIOSpark());
        wrist = new Wrist(new WristIOSpark());
        climber = new Climber(new ClimberIOSpark());
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
        endEffecter = new EndEffecter(new EndEffecterIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climber(new ClimberIOSim());
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
        endEffecter = new EndEffecter(new EndEffecterIO() {});
        wrist = new Wrist(new WristIO() {});
        climber = new Climber(new ClimberIO() {});
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
    // configureButtonBindings();
    updateDashboard();
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
    SmartDashboard.putData("HopperUp", hopper.runPercent(0.1));
    SmartDashboard.putData("HopperDown", hopper.runPercent(-0.1));

    // Advanced Commands
    SmartDashboard.putData(
        "IntakeCoral", endEffecter.runPercent(0.5).until(endEffecter::isTriggered));
    SmartDashboard.putData("WristHomePos", wrist.setWristPosition(WristConstants.homeSetpoint));
    SmartDashboard.putData("WristL23Pos", wrist.setWristPosition(WristConstants.coralL23Setpoint));
    SmartDashboard.putData("WristL4Pos", wrist.setWristPosition(WristConstants.coralL4Setpoint));
    SmartDashboard.putData("WristAlgaePos", wrist.setWristPosition(WristConstants.aglaeSetpoint));
    SmartDashboard.putData(
        "WristProcPos", wrist.setWristPosition(WristConstants.processorSetpoint));
    SmartDashboard.putData("WristNetPos", wrist.setWristPosition(WristConstants.netSetPoint));
    SmartDashboard.putData(
        "ElevatorHomePos", elevator.setElevatorPosition(ElevatorConstants.homeSetpoint, true));
    SmartDashboard.putData(
        "ElevatorCoralL2Pos",
        elevator.setElevatorPosition(ElevatorConstants.coralL2Setpoint, true));
    SmartDashboard.putData(
        "ElevatorCoralL3Pos",
        elevator.setElevatorPosition(ElevatorConstants.coralL3Setpoint, true));
    SmartDashboard.putData(
        "ElevatorCoralL4Pos",
        elevator.setElevatorPosition(ElevatorConstants.coralL4Setpoint, true));
    SmartDashboard.putData(
        "ElevatorAlgaeL2Pos",
        elevator.setElevatorPosition(ElevatorConstants.aglaeL2Setpoint, true));
    SmartDashboard.putData(
        "ElevatorAlgaeL3Pos",
        elevator.setElevatorPosition(ElevatorConstants.aglaeL3Setpoint, true));
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
