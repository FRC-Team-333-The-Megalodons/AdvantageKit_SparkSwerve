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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.Metric;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  public static Pose3d visionRobotPose;
  public static int visionTagId = -1;
  public static long visionTagLastSeen = -1;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public int getFudicialId() {
    int id = 0;
    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        id = tagId;
      }
    }
    return id;
  }

  final String visPer_comp = "VisionPeriodic";

  @Override
  public void periodic() {
    Metric ioLoop_metric = new Metric(visPer_comp, "IO_UpdateLoop"),
        camLoop_metric = new Metric(visPer_comp, "Camera_Loop"),
        poseLoop_metric = new Metric(visPer_comp, "PoseLoop"),
        tagLoop_metric = new Metric(visPer_comp, "TagLoop"),
        reefAngle_metric = new Metric(visPer_comp, "ReefAngleCalc");

    ioLoop_metric.start();
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      Logger.recordOutput("Fudicial ID HK", getFudicialId());
    }
    ioLoop_metric.stop();

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Looping over multiple cameras, we can get multiple "poses" based on each effective tag.
    // Our goal is to use the pose from the "closest" tag - but, two different cameras might 
    //  have two different tags that are their closest.
    // To manage this, we can 

    // Loop over cameras
    camLoop_metric.start();
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        double x = observation.pose().getX(), y = observation.pose().getY();
        boolean isPoseSane = /* somehow, origin value get in here. that needs to go away. */
            x != 0 && y != 0 && observation.pose().getRotation().getAngle() != 0;
        // observation.tagCount() > 0 &&                         // Must have at least one tag
        // found
        /*
        observation.ambiguity() <= maxAmbiguity // Cannot be high ambiguity
            && Math.abs(observation.pose().getZ())
                <= maxZError // Must have realistic Z coordinate
            && x > 0
            && x < aprilTagLayout.getFieldLength() // Must be within the field boundaries
            && y > 0
            && y < aprilTagLayout.getFieldWidth() // Must be within the field boundaries
        ;
        */

        // Add pose to log
        robotPoses.add(observation.pose());
        if (!isPoseSane) {
          robotPosesRejected.add(observation.pose());
          continue;
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        // if (rejectPose) {
        //   continue;
        // }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }
    camLoop_metric.stop();

    // We can potentially end up with multiple poses, and be unable to determine which is correct.
    // In that case, we can use the physical odometry as a reference to help us "break the tie" -
    // Of the poses, the one with the closes Position2d to the drivetrain's (motors+gyro) odometry
    //  is (probably) the correct one!
    Pose2d drivetrainPose2d = Drive.estimatedPose2d;
    Logger.recordOutput("Vision/Summary/DriveTrainPose", drivetrainPose2d);
    Pose3d bestRobotPose = null;
    double bestRobotPoseScore = Double.MAX_VALUE;
    poseLoop_metric.start();
    for (int i = 0; i < allRobotPosesAccepted.size(); ++i) {
      Pose3d pose3d = allRobotPosesAccepted.get(i);
      double score = getScaledPose2dDiff(drivetrainPose2d, pose3d.toPose2d());
      if (score < bestRobotPoseScore) {
        bestRobotPose = pose3d;
        bestRobotPoseScore = score;
      }
    }
    poseLoop_metric.stop();

    visionRobotPose = bestRobotPose;

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    // Logger.recordOutput("Vision/Summary/BestRobotPose", bestRobotPose);

    // Find the closest tag.
    if (bestRobotPose != null) {
      double closestTagDistance = Double.MAX_VALUE;
      int closestTagId = -1;
      tagLoop_metric.start();
      for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
        for (int tagId : inputs[cameraIndex].tagIds) {
          Optional<Pose3d> tagPoseOpt = aprilTagLayout.getTagPose(tagId);
          if (tagPoseOpt.isPresent()) {
            Pose3d tagPose = tagPoseOpt.get();
            double distance = tagPose.getTranslation().getDistance(bestRobotPose.getTranslation());
            if (distance < closestTagDistance) {
              closestTagDistance = distance;
              closestTagId = tagId;
            }
          }
        }
      }
      tagLoop_metric.stop();
      if (closestTagId >= 0) {
        visionTagId = closestTagId;
        visionTagLastSeen = System.currentTimeMillis();
      }
    }

    Logger.recordOutput("Vision/Summary/ClosestSeenTag/TagId", visionTagId);
    Logger.recordOutput("Vision/Summary/ClosestSeenTag/LastSeen", visionTagLastSeen);

    // This call is inexpensive (i think), and we're using it purely for logging;
    // But if things get laggy, delete this line!
    reefAngle_metric.start();
    Drive.reefDriveAngle(this);
    reefAngle_metric.stop();

    ioLoop_metric.logThrottled();
    camLoop_metric.logThrottled();
    poseLoop_metric.logThrottled();
    tagLoop_metric.logThrottled();
    reefAngle_metric.logThrottled();

    // Logger.recordOutput("Vision/Summary/BestRobotPose", bestRobotPose);
  }

  public static int getCurrentVisionTagId() {
    final long TAG_SEEN_DEADBAND = 2000;
    if (Vision.visionTagId >= 0
        && Vision.visionTagLastSeen > 0
        && System.currentTimeMillis() - Vision.visionTagLastSeen < TAG_SEEN_DEADBAND) {
      return Vision.visionTagId;
    }
    return -1;
  }

  private static Double hypotenuse = null;

  public double getFieldHypotenuse() {
    // We're using a static variable to cache the result so we don't do the expensive
    //  pythagorean theorem calc for hypotenuse every time.
    if (hypotenuse == null) {
      double fieldLength = aprilTagLayout.getFieldLength();
      double fieldWidth = aprilTagLayout.getFieldWidth();
      hypotenuse = Math.sqrt(Math.pow(fieldLength, 2) + Math.pow(fieldWidth, 2));
    }
    return hypotenuse;
  }

  public double getScaledPose2dDiff(Pose2d a, Pose2d b) {
    // We're effectively "scoring" the difference, as a combination of Translation and Rotation.
    // Balancing the delta between the rotation and translation is a bit hard. We need to scale
    // distance relative to the field size (i.e. hypotenuse)
    double scaledRotationDelta = Math.abs(a.relativeTo(b).getRotation().getDegrees()) / 180;
    double scaledTranslationDelta =
        Math.abs(a.getTranslation().getDistance(b.getTranslation())) / getFieldHypotenuse();

    return scaledRotationDelta + scaledTranslationDelta;
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
