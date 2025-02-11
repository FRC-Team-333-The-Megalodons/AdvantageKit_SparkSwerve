// Copyright 2021-2024 FRC 6328
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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.assistedDrive.DriveToClosestReef;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private Intake intake;
  private final LEDStrip led;
  private SwerveDriveSimulation driveSimulation = null;
  private DriveCommands driveCommands;

  public static List<PhotonTrackedTarget> frontCameraTargets = new ArrayList<>();
  public static List<PhotonTrackedTarget> backCameraTargets = new ArrayList<>();

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
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});

        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        intake = new Intake(new IntakeIOSpark());

        led = new LEDStrip();

        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        intake = new Intake(new IntakeIOSim());
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
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        intake = new Intake(new IntakeIO() {});

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

    // Configure the button bindings
    configureButtonBindings();
    addCommandsToDashboard();
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

    // Default roller command, control with triggers
    // Commented out until something can be plugged into Can ID 5
    intake.setDefaultCommand(
        intake.runTeleop(() -> controller.getR2Axis(), () -> controller.getL2Axis()));

    // Running wrist encoder
    controller.circle().whileTrue(intake.runIntake(IntakeConstants.setPoint));

    // Eject game pieve when triangle is held
    // Commented out until something can be plugged into Can ID 5

    controller
        .triangle()
        .whileTrue(
            drive.isNearCoralStation()
                ? intake
                    .runPercent(-0.333)
                    .alongWith(new RunCommand(() -> LEDStrip.setLEDs(Color.kRed)))
                    .until(intake::isTriggered)
                    .andThen(new RunCommand(() -> LEDStrip.setLEDs(Color.kGreen)))
                : intake.runPercent(0))
        .onFalse(new RunCommand(() -> LEDStrip.setLEDs(Color.kBlack)));

    // Lock to 0°
    controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> drive.isRed() ? controller.getLeftY() : -controller.getLeftY(),
                () -> drive.isRed() ? controller.getLeftX() : -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern
    controller.square().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0°
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    controller.options().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    controller
        .L1()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> drive.isRed() ? controller.getLeftY() : -controller.getLeftY(),
                () -> drive.isRed() ? controller.getLeftX() : -controller.getLeftX(),
                () -> vision.getTargetX(0)));
    controller.R1().whileTrue(DriveCommands.aimAtTheTarget(vision, drive));

    controller.create().whileTrue(new DriveToClosestReef(drive));
  }

  static List<Integer> blueReefTags = new ArrayList<>(Arrays.asList(17, 18, 19, 20, 21, 22));
  static List<Integer> redReefTags = new ArrayList<>(Arrays.asList(6, 7, 8, 9, 10, 11));

  static List<Integer> blueCoralStationTags = new ArrayList<>(Arrays.asList(12, 13));
  static List<Integer> redCoralStationTags = new ArrayList<>(Arrays.asList(1, 2));

  static List<Integer> blueProcessorTags = new ArrayList<>(Arrays.asList(3));
  static List<Integer> redProcessorTags = new ArrayList<>(Arrays.asList(16));

  static List<Integer> blueBargeTags = new ArrayList<>(Arrays.asList(14, 4));
  static List<Integer> redBargeTags = new ArrayList<>(Arrays.asList(15, 5));

  public static boolean amBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    // In testing mode, we'll act as if we're the blue alliance.
  }

  public static Pose2d getClosestReefPose() {
    double closestDistance = Double.MAX_VALUE;
    PhotonTrackedTarget closestValidTarget = null;
    for (PhotonTrackedTarget target : frontCameraTargets) {
      Integer tagId = target.getFiducialId();
      List<Integer> reefTags = amBlueAlliance() ? blueReefTags : redReefTags;
      if (reefTags.contains(tagId)) {
        double distance =
            PhotonUtils.calculateDistanceToTargetMeters(
                0.2, // Mesaured with a tape measure, how high is the camera from the ground on our
                // robot
                1.4, // How high up off the ground is an April tag, should be from Game Manual
                Units.degreesToRadians(
                    -30), // TODO: I have no idea what this number is! It's something from our robot
                // presumably.
                Units.degreesToRadians(target.getPitch()));

        if (distance < closestDistance) {
          closestDistance = distance;
          closestValidTarget = target;
        }
      }
    }

    if (closestValidTarget == null) {
      return null;
    }

    if (VisionConstants.aprilTagLayout.getTagPose(closestValidTarget.getFiducialId()).isPresent()) {
      Pose3d robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              closestValidTarget.getBestCameraToTarget(),
              aprilTagLayout.getTagPose(closestValidTarget.getFiducialId()).get(),
              new Transform3d()); // TODO: There should be a class that describes the physical
      // robot's center relative to the camera.
      return robotPose.toPose2d();
    }

    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void addCommandsToDashboard() {
    SmartDashboard.putData(
        "Blue Segmented",
        new RunCommand(() -> led.makeSegmentColorCommand(Color.kBlue, LEDStrip.getSegment(4, 3))));
    SmartDashboard.putData(
        "Blue Whole", new RunCommand(() -> led.makeWholeColorCommand(Color.kAqua)));
  }
}
