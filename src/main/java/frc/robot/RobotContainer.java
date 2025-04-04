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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutomatedCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffecterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endEffecter.EndEffecter;
import frc.robot.subsystems.endEffecter.EndEffecterConstants;
import frc.robot.subsystems.endEffecter.EndEffecterIO;
import frc.robot.subsystems.endEffecter.EndEffecterIOSim;
import frc.robot.subsystems.endEffecter.EndEffecterIOSpark;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.ramp.RampConstants;
import frc.robot.subsystems.ramp.RampIO;
import frc.robot.subsystems.ramp.RampIOSim;
import frc.robot.subsystems.ramp.RampIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.GlobalConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer { // Subsystems
  public final Drive drive;
  private final Elevator elevator;
  private final EndEffecter endEffecter;
  private final Wrist wrist;
  public final Climber climber;

  private final Ramp ramp;
  private final Vision vision;

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final boolean startInManualMode = false;
  private final boolean isInSoloDrivingMode = false;

  private double applyJoystickAllianceAndLimits(double value) {
    /*
    if (drive.isRed()) {
      value *= -1; // Flip the direction if we're not Red.
    }
    */

    // Remove this code if you want to remove Speed Limiting when elevator up
    if (Elevator.isPastSlowdownHeight) {
      // If the elevator is higher than the slow-limiter height, cut the joystick in half.
      value /= 2;
    }
    return value;
  }

  private double getDriverLeftY() {
    return applyJoystickAllianceAndLimits(driverController.getLeftY());
  }

  private double getDriverLeftX() {
    return applyJoystickAllianceAndLimits(driverController.getLeftX());
  }

  private double getDriverRightX() {
    return -driverController.getRightX();
  }

  private void configureInitialControllerBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> getDriverLeftY(), () -> getDriverLeftX(), () -> getDriverRightX()));
    configureDriverControllerBindings();
    if (startInManualMode) {
      configureOperatorControllerManualModeBindings();
    } else {
      configureOperatorControllerSmartModeBindings();
    }
  }

  private void configureDriverControllerBindings() {
    driverController
        .R3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> getDriverLeftY(),
                () -> getDriverLeftX(),
                () -> getDriverRightX(), // only used if no valid reef angle
                () -> Rotation2d.fromDegrees(Drive.reefDriveAngle(vision))));

    driverController.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // TODO: Figure out how to fix the crazy jumping with the Precise Drive to Reef Command
    // driverController.povUp().whileTrue(DriveCommands.generateDriveToReefCommand('M'));
    // driverController.povLeft().whileTrue(DriveCommands.generateDriveToReefCommand('L'));
    // driverController.povRight().whileTrue(DriveCommands.generateDriveToReefCommand('R'));
    driverController.povUp().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('M', drive));
    driverController
        .povLeft()
        .whileTrue(DriveCommands.generatePreciseDriveToReefCommand('L', drive));
    driverController
        .povRight()
        .whileTrue(DriveCommands.generatePreciseDriveToReefCommand('R', drive));
    // driverController.L1().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('M', drive));
    // driverController.R1().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('M', drive));
    // driverController.L2().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('L', drive));
    // driverController.R2().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('R', drive));

    if (isInSoloDrivingMode) {
      driverController
          .PS()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.setPose(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
    } else {
      driverController
          .povLeft()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.setPose(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
      driverController.L1().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('M', drive));
      driverController.R1().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('M', drive));
      driverController.L2().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('L', drive));
      driverController.R2().whileTrue(DriveCommands.generatePreciseDriveToReefCommand('R', drive));
    }
  }

  public void removeOperatorControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings();
  }

  public void configureOperatorControllerManualModeBindings() {
    // wrist.setDefaultCommand(wrist.setWristPosition(WristConstants.homeSetpoint));
    // ramp.setDefaultCommand(ramp.runPercent(RampConstants.speed));
    if (isInSoloDrivingMode) {
      driverController
          .povUp()
          .whileTrue(elevator.runPercent(ElevatorConstants.speed).until(elevator::upperLimit));
      driverController
          .povDown()
          .whileTrue(elevator.runPercent(-ElevatorConstants.speed).until(elevator::lowerLimit));
      driverController.povLeft().whileTrue(climber.getClimberInCommand());
      driverController.povRight().whileTrue(climber.getClimberOutCommand(ramp));
      driverController.create().whileTrue(ramp.runPercent(RampConstants.speed));
      driverController.options().whileTrue(ramp.runPercent(-RampConstants.speed));

      driverController.L1().whileTrue(wrist.runPercent(WristConstants.speed));
      driverController.R1().whileTrue(wrist.runPercent(-WristConstants.speed));

      driverController.L2().whileTrue(endEffecter.runPercent(-EndEffecterConstants.speed));
      driverController.R2().whileTrue(endEffecter.runPercent(EndEffecterConstants.speed));

    } else {
      operatorController
          .povUp()
          .whileTrue(elevator.runPercent(ElevatorConstants.speed).until(elevator::upperLimit));
      operatorController
          .povDown()
          .whileTrue(elevator.runPercent(-ElevatorConstants.speed).until(elevator::lowerLimit));
      driverController.povLeft().whileTrue(climber.getClimberInCommand());
      driverController.povRight().whileTrue(climber.getClimberOutCommand(ramp));
      operatorController.create().whileTrue(ramp.runPercent(RampConstants.speed));
      operatorController.options().whileTrue(ramp.runPercent(-RampConstants.speed));

      operatorController.L1().whileTrue(wrist.runPercent(WristConstants.speed));
      operatorController.R1().whileTrue(wrist.runPercent(-WristConstants.speed));

      operatorController.L2().whileTrue(endEffecter.runPercent(-EndEffecterConstants.speed));
      operatorController.R2().whileTrue(endEffecter.runPercent(EndEffecterConstants.speed));
    }
  }

  public void configureOperatorControllerSmartModeBindings() {
    // wrist.setDefaultCommand(wrist.setWristPosition(WristConstants.homeSetpoint));
    // ramp.setDefaultCommand(ramp.runPercent(RampConstants.speed));
    if (isInSoloDrivingMode) {
      driverController
          .L2()
          .whileTrue(
              AutomatedCommands.homeCommand(wrist, elevator, ramp)
                  .alongWith(
                      EndEffecterCommands.runEndEffecterForward(endEffecter)
                          .until(endEffecter::isTriggered)));

      driverController.R2().whileTrue(EndEffecterCommands.runEndEffecterForward(endEffecter));
      driverController.touchpad().whileTrue(AutomatedCommands.rampGoToIntakePosition(endEffecter));

      driverController
          .options() // get ready
          .whileTrue(climber.getClimberOutCommand(ramp));

      driverController
          .create() // actually climb
          .whileTrue(climber.getClimberInCommand());

      driverController
          .triangle()
          .whileTrue(AutomatedCommands.coralL4Command(endEffecter, wrist, elevator));
      driverController
          .circle()
          .whileTrue(AutomatedCommands.coralL3Command(endEffecter, wrist, elevator));
      driverController
          .square()
          .whileTrue(AutomatedCommands.coralL2Command(endEffecter, wrist, elevator));

      driverController.cross().whileTrue(EndEffecterCommands.runEndEffecterBackward(endEffecter));

      driverController
          .povUp()
          .whileTrue(
              AutomatedCommands.netCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      driverController
          .povDown()
          .whileTrue(
              AutomatedCommands.processorCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      driverController
          .povLeft()
          .whileTrue(
              AutomatedCommands.algaeL2Command(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      driverController
          .povRight()
          .whileTrue(
              AutomatedCommands.algaeL3Command(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      driverController
          .touchpad()
          .whileTrue(
              AutomatedCommands.homeWithAlgaeCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));

      driverController
          .R1()
          .whileTrue(AutomatedCommands.rampIntakeCommand(ramp, RampConstants.speed));
      driverController
          .L1()
          .whileTrue(AutomatedCommands.rampIntakeCommand(ramp, -RampConstants.speed));
    } else {
      //   operatorController
      //       .touchpad()
      //       .whileTrue(AutomatedCommands.rampGoToIntakePosition(ramp, endEffecter));
      operatorController
          .L2()
          .whileTrue(
              AutomatedCommands.homeCommand(wrist, elevator, ramp)
                  .alongWith(
                      EndEffecterCommands.runEndEffecterForward(endEffecter)
                          .until(endEffecter::isTriggered)));

      operatorController.R2().whileTrue(EndEffecterCommands.runEndEffecterForward(endEffecter));

      driverController.options().whileTrue(climber.getClimberOutCommand(ramp));

      driverController.create().whileTrue(climber.getClimberInCommand());

      operatorController
          .triangle()
          .whileTrue(AutomatedCommands.coralL4Command(endEffecter, wrist, elevator));
      operatorController
          .circle()
          .whileTrue(AutomatedCommands.coralL3Command(endEffecter, wrist, elevator));
      operatorController
          .square()
          .whileTrue(AutomatedCommands.coralL2Command(endEffecter, wrist, elevator));

      //
      // operatorController.cross().whileTrue(EndEffecterCommands.runEndEffecterBackward(endEffecter));

      operatorController
          .povUp()
          .whileTrue(
              AutomatedCommands.netCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      operatorController
          .povDown()
          .whileTrue(
              AutomatedCommands.processorCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      operatorController
          .povLeft()
          .whileTrue(
              AutomatedCommands.algaeL2Command(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      operatorController
          .povRight()
          .whileTrue(
              AutomatedCommands.algaeL3Command(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));
      operatorController
          .cross()
          .whileTrue(
              AutomatedCommands.homeWithAlgaeCommand(endEffecter, wrist, elevator)
                  .alongWith(EndEffecterCommands.runEndEffecterBackward(endEffecter)));

      operatorController.L1().whileTrue(AutomatedCommands.intakeCoralAgain(endEffecter));

      operatorController
          .R1()
          .whileTrue(
              EndEffecterCommands.runEndEffecterBackward(endEffecter)
                  .alongWith(AutomatedCommands.rampIntakeCommand(ramp, -RampConstants.speed)));

      //  operatorController.PS().whileTrue(ramp.runPercent(RampConstants.speed));

      operatorController
          .R3()
          .whileTrue(AutomatedCommands.netLobCommand(endEffecter, wrist, elevator));
    }
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
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        endEffecter = new EndEffecter(new EndEffecterIOSpark());
        wrist = new Wrist(new WristIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        ramp = new Ramp(new RampIOTalonFX());
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
        ramp = new Ramp(new RampIOSim());
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
        ramp = new Ramp(new RampIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Named Commands
    NamedCommands.registerCommand(
        "ScoreCoral",
        EndEffecterCommands.runEndEffecterForward(endEffecter)
            .onlyWhile(endEffecter::isTriggered)
            .andThen(wrist.setWristPosition(WristConstants.coralL23Setpoint))
            .until(wrist::atL3Setpoint)); // onlyIf(endEffecter::isTriggered));
    NamedCommands.registerCommand("IntakeCoral", AutomatedCommands.autoIntakeCoral(endEffecter));
    NamedCommands.registerCommand(
        "RunRamp", AutomatedCommands.rampIntakeCommand(ramp, RampConstants.speed));
    NamedCommands.registerCommand("HomePos", AutomatedCommands.autoHomeCommand(wrist, elevator));
    NamedCommands.registerCommand(
        "CoralL4Position", AutomatedCommands.autoScoreL4(endEffecter, wrist, elevator));
    NamedCommands.registerCommand(
        "AlgaeL2Position", AutomatedCommands.autoAlgaeL2Command(endEffecter, wrist, elevator));
    NamedCommands.registerCommand(
        "AlgaeInNet", AutomatedCommands.autoNetCommand(endEffecter, wrist, elevator));

    new EventTrigger("l4 position").whileTrue(Commands.print("Going to L4 position"));
    new EventTrigger("l3 position").whileTrue(Commands.print("Going to L3 position"));
    new EventTrigger("l2 position").whileTrue(Commands.print("Going to L2 position"));
    new EventTrigger("score coral").whileTrue(Commands.print("Score Coral"));
    new EventTrigger("home position").whileTrue(Commands.print("Going Home"));
    new EventTrigger("intake coral")
        .whileTrue(
            Commands.print("Intaking Coral...")
                .until(endEffecter::isTriggered)
                .andThen(Commands.print("Coral Intaked!")));

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
    configureInitialControllerBindings();
    SmartDashboard.putBoolean(GlobalConstants.MANUAL_MODE_KEY, false);
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
    SmartDashboard.putData("ExtendClimber", climber.getClimberOutCommand(ramp, Climber.QUARTER_SPEED));
    // .alongWith(climber.runServo(0.5, 90)));
    SmartDashboard.putData("RampServoOut", ramp.runServoAtSpeed(1.0));
    SmartDashboard.putData("RampServoIn", ramp.runServoAtSpeed(-1.0));
    SmartDashboard.putData("RetractClimber", climber.getClimberInCommand(Climber.QUARTER_SPEED));

    SmartDashboard.putData("ServoDown", climber.runServoToPosition(Climber.SERVO_UNLOCKED));
    SmartDashboard.putData("ServoUp", climber.runServoToPosition(Climber.SERVO_LOCKED));

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

  public Command getPeriodicCommand() {
    return AutomatedCommands.rampIntakeCommand(ramp, RampConstants.speed);
  }
}
