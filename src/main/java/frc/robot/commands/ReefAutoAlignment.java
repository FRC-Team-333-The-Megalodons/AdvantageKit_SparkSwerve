package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ReefAutoAlignment extends SequentialCommandGroup {
  /**
   * creates a precise auto-alignment command NOTE: AutoBuilder must be configured! the command has
   * two steps: 1. path-find to the target pose, roughly 2. accurate auto alignment
   */
  public ReefAutoAlignment(
      PathConstraints constraints,
      HolonomicDriveController holonomicDriveController,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<ChassisSpeeds> robotRelativeSpeedsOutput,
      Subsystem driveSubsystem,
      char side) {
    /* tolerance for the precise approach */
    holonomicDriveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(10)));
    Supplier<Pose2d> targetPoseSupplier = () -> DriveCommands.getNearestReefSidePose(side);
    Supplier<Rotation2d> rotationSupplier =
        () -> DriveCommands.getBestRotation2dTarget(robotPoseSupplier);
    final Command pathFindToTargetRough =
        DriveCommands.generateDriveToReefCommand(side); // Maybe increase goal end velocity?
    final Command preciseAlignment =
        new FunctionalCommand(
            () -> {},
            () ->
                robotRelativeSpeedsOutput.accept(
                    holonomicDriveController.calculate(
                        robotPoseSupplier.get(),
                        targetPoseSupplier.get(),
                        0,
                        // targetPoseSupplier.get().getRotation())),
                        // robotPoseSupplier.get().getRotation())),
                        rotationSupplier.get())),
            (interrupted) -> robotRelativeSpeedsOutput.accept(new ChassisSpeeds()),
            holonomicDriveController::atReference);

    super.addCommands(pathFindToTargetRough);
    super.addCommands(preciseAlignment);

    super.addRequirements(driveSubsystem);
  }
}
