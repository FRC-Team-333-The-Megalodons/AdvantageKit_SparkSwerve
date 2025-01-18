// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.assistedDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToClosestReef extends Command {
  private Drive drive;
  private Command pathFindingCommand;
  /** Creates a new RunCoralIntake. */
  public DriveToClosestReef(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = RobotContainer.getClosestReefPose();

    if (pose == null) {
      DriverStation.reportError("Could not find any Reef Pose.", true);
      System.out.println("Could not find any Reef Pose.");

      return;
    }

    DriverStation.reportError("Found reef pose!", true);
    System.out.println("Found reef pose!");

    PathConstraints constraints =
        new PathConstraints(
            3.0, // max linear velocity
            4.0, // max linear acceleration
            Units.degreesToRadians(540), // max rotation per second
            Units.degreesToRadians(720)); // max angular acceleration

    pathFindingCommand =
        AutoBuilder.pathfindToPose(
            pose, // target pose to drive to
            constraints, // limits on velocity/spin/acceleration
            0.0); // goal end velocity in m/s
    // 0.0); // r

    DriverStation.reportError("Initialized path to reef", true);
    System.out.println("Initialized path to reef");
    pathFindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // do the hard math here to figure out how to do it

    if (pathFindingCommand == null) {
      DriverStation.reportError("PathFindingCommand is null", true);
      System.out.println("PathFindingCommand is null");
      return;
    }

    pathFindingCommand.execute();

    // This is basically the limelight sample code, but the reality is that we have a fully
    // field-aware robot and can use pathplanner.
    /*
        final double CAMERA_ANGLE_OFFSET_TO_ACTUAL_FRONT_OF_ROBOT_deg = 0; // TODO: Measure this, also handle the back camera
        final double CAMERA_DISTANCE_OFFSET_TO_ACTUAL_FRONT_OF_ROBOT_m = 0; // TODO: Measure this, also handle the back camera
        final double VISION_TURN_kP = 0.1; // TODO: Tune this!
        final double VISION_STRAFE_kP = 0.1; // TODO: Tune this!

        final double kMaxAngularSpeed = 1.4; // TODO: Tune this!
        final double kMaxLinearSpeed = 0.4; // TODO: Tune this!
        double turn = (CAMERA_ANGLE_OFFSET_TO_ACTUAL_FRONT_OF_ROBOT_deg - target.getYaw()) * VISION_TURN_kP * kMaxAngularSpeed;
        double forward = (CAMERA_DISTANCE_OFFSET_TO_ACTUAL_FRONT_OF_ROBOT_m - targetDistance) * VISION_STRAFE_kP * kMaxLinearSpeed;
    */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathFindingCommand == null) {
      return;
    }
    pathFindingCommand.end(interrupted);
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathFindingCommand == null) {
      return true;
    }
    return pathFindingCommand.isFinished();
  }
}
