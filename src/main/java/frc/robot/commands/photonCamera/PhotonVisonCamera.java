// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class PhotonVisonCamera extends SubsystemBase {
  private PhotonCamera camera;
  public static int fudicialId;
  public static double yaw;
  public static double pitch;
  public static boolean targetVisible;
  // public static AprilTagFieldLayout aprilTagLayout;
  // public static Rotation2d targetYaw;
  // public static Pose3d robotPose3d;
  // private double target;
  // private int fudicialId;

  /** Creates a new PhotonVisonCamera. */
  public PhotonVisonCamera() {
    camera = new PhotonCamera("limelight_rear");
    // aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    this.yaw = 0.0;
    this.fudicialId = 0;
    this.pitch = 0.0;
    this.targetVisible = false;
    // this.targetYaw = Rotation2d.fromDegrees(0);

    // for (var result : results) {
    //   if (result.hasTargets()) {
    //     target = result.getBestTarget().getYaw();
    //     fudicialId = result.getBestTarget().getFiducialId();
    //   }
    // }
  }

  // public double getIdYaw() {
  //   var results = camera.getAllUnreadResults();
  //   double yaw = 0;
  //   for (var result : results) {
  //     if (result.getBestTarget().getFiducialId() == 1) {
  //       yaw = result.getBestTarget().getYaw();
  //     }
  //   }
  //   return yaw;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.fudicialId = 0;
    this.yaw = 0;
    // this.yaw = 0.0;
    this.pitch = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          // if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {}
          //   Pose2d robotPose = PhotonUtils.estimateFieldToCamera(null, null);
          // }
          // targetYaw = PhotonUtils.getYawToPose(null, null);
          this.fudicialId = target.getFiducialId();
          this.yaw = target.getYaw();
          this.pitch = target.getPitch();

          this.targetVisible = true;
        }
      } else {
        this.targetVisible = false;
      }
    }

    SmartDashboard.putNumber("LimelightYaw ", this.yaw);
    SmartDashboard.putNumber("Id ", (double) this.fudicialId);
    SmartDashboard.putBoolean("IsVisible ", targetVisible);
    Logger.recordOutput("Logged Fudicial Id ", fudicialId);
  }

  public static double getYaw() {
    return yaw;
  }

  public static boolean IsVisible() {
    return targetVisible;
  }
}
