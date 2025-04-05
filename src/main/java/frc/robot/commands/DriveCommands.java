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

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriverConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Metric;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static Metric joyDrive_metric = new Metric("JoystickDrive", "execute");
  public static Metric joyDriveAngle_metric = new Metric("JoystickDriveAngle", "execute");
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          joyDrive_metric.start();
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Blue;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
          joyDrive_metric.stop();
          joyDrive_metric.logThrottled();
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              joyDriveAngle_metric.start();
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              boolean respectAutoDriveAngle = true;
              // Decide if we've got valid data from the Automatic Rotation system.
              if (rotationSupplier.get().getDegrees()
                  == DriverConstants.MAGIC_INVALID_DEGREES_NUMBER) {
                respectAutoDriveAngle = false;
              }

              // Calculate angular speed
              double omega;

              if (respectAutoDriveAngle) {
                omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
              } else {
                // This means we didn't actually get valid degrees, and we need to just trust the
                // joystick.
                omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);
              }

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Blue;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              joyDriveAngle_metric.stop();
              joyDriveAngle_metric.logThrottled();
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Pose2d getNearestReefSidePose(char side) {
    int reefTag = Drive.odometryTagId;
    Logger.recordOutput("Vision/Summary/DriveToReef/ReefTag", reefTag);
    double tagAngle = Drive.reefDriveAngleByTagId(reefTag);
    Logger.recordOutput("Vision/Summary/DriveToReef/ReefAngle", tagAngle);

    Logger.recordOutput(
        "Vision/Summary/DriveToReef/LastCalledTimestamp", System.currentTimeMillis() % 1000);
    if (tagAngle == DriverConstants.MAGIC_INVALID_DEGREES_NUMBER) {
      // we know that this isn't a valid Reef Tag for our alliance, so bail out.
      return null;
    }

    Optional<Pose3d> tagPose3d = VisionConstants.aprilTagLayout.getTagPose(reefTag);
    if (tagPose3d.isEmpty()) {
      // for some reason, we couldn't find the particular tag in the field layout.
      return null;
    }

    // If we made it this far, we know we found the real pose of the real tag
    Pose2d tagPose = tagPose3d.get().toPose2d();

    // This is the left/right distance from the Tag center we want to be for placing the coral on
    // the left/right posts.
    // (If M is passed, we're getting the Ball, so we want to be centered.)
    // double sideDistance = 0.25;
    double sideDistance = 0.09;
    if (side == 'M') {
      // sideDistance = 0.1; // If we're staying in the middle, there's no side distance.
      sideDistance = 0.0; // If we're staying in the middle, there's no side distance.
    } else if (side == 'L') {
      // sideDistance = -0.09; // If going left, flip the side direction
      sideDistance = -0.09; // If going left, flip the side direction
    }

    // This is the buffer to the tag position (i.e. don't slam into wall)
    double forwardDistance = 0.56;

    // 1. Define the movement (forward by forwardDistance units, side by sideDistance units)
    double forwardX = forwardDistance * Math.cos(tagPose.getRotation().getRadians());
    double forwardY = forwardDistance * Math.sin(tagPose.getRotation().getRadians());
    double sideX = -sideDistance * Math.sin(tagPose.getRotation().getRadians());
    double sideY = sideDistance * Math.cos(tagPose.getRotation().getRadians());

    Translation2d newTranslation =
        tagPose
            .getTranslation()
            .plus(new Translation2d(forwardX, forwardY))
            .plus(new Translation2d(sideX, sideY));
    Pose2d newPose = new Pose2d(newTranslation, Rotation2d.fromDegrees(tagAngle));
    return newPose;
  }

  public static Command getDriveToReefSideCommand(char side) {
    Pose2d targetPose = getNearestReefSidePose(side);

    // If targetPose is null, it means we couldn't figure out where to go.
    if (targetPose == null) {
      return Commands.print("Could not find Reef Side Target");
    }

    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(targetPose, PathConstraints.unlimitedConstraints(12.0), 0.0);

    return pathfindingCommand;
  }

  public static Command generatePreciseDriveToReefCommand(char side, Drive drive) {
    final double kMaxAngularVelocity = 60;
    final double kMaxAngularAcceleration = 60;
    final double kP = 3.5, kI = 0.7, kD = 0.0;
    final double kPtheta = 0.5, kItheta = 0.1, kDtheta = 0.2;
    Supplier<Pose2d> robotPoseSupplier = () -> Drive.estimatedPose2d;
    HolonomicDriveController controller =
        new HolonomicDriveController(
            new PIDController(kP, kI, kD), // X controller
            new PIDController(kP, kI, kD), // Y controller
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)));
    /*
    controller.setTolerance(
        new Pose2d(
            0.05, // Keep X tolerance the same (5 cm)
            0.05, // Keep Y tolerance the same (5 cm)
            Rotation2d.fromDegrees(10))); // Increase rotational tolerance to 10 degrees
            */

    PathConstraints constraints = new PathConstraints(3.0, 4.0, 0, 0);
    return new ReefAutoAlignment(
        constraints, controller, robotPoseSupplier, drive.getChassisSpeedsConsumer(), drive, side);
  }

  public static Rotation2d getBestRotation2dTarget(Supplier<Pose2d> robotPoseSupplier) {
    Rotation2d output = null;
    try {
      int tagId = Drive.odometryTagId;
      if (tagId >= 0) {
        double degrees = Drive.reefDriveAngleByTagId(tagId);
        output = Rotation2d.fromDegrees(degrees);
      }
      if (output == null) {
        // If we couldn't figure it out, just return the direction we're facing today
        if (robotPoseSupplier.get() != null) {
          output = robotPoseSupplier.get().getRotation();
        }
      }
    } catch (Exception e) {
    }

    if (output == null) {
      // Anything is better than nothing?
      output = Rotation2d.fromDegrees(0);
    }

    return output;
  }

  static Metric reefInit_metric = new Metric("DriveToReef", "initTime");
  static Metric reefExec_metric = new Metric("DriveToReef", "execTime");

  public static Command generateDriveToReefCommand(char side) {
    return new Command() {
      Command command = null;

      @Override
      public void initialize() {
        reefInit_metric.start();
        command = DriveCommands.getDriveToReefSideCommand(side);
        command.initialize();
        reefInit_metric.stop();
      }

      @Override
      public void execute() {
        reefExec_metric.start();
        command.execute(); // Execute the generated command
        reefExec_metric.stop();
      }

      @Override
      public void end(boolean interrupted) {
        reefInit_metric.log();
        reefExec_metric.log();
        command.end(interrupted);
      }

      @Override
      public boolean isFinished() {
        return command.isFinished();
      }
    };
  }
}
