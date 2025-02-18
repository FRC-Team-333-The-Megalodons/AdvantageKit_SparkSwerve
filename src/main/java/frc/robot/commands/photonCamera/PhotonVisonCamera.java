// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class PhotonVisonCamera extends SubsystemBase {
  private PhotonCamera camera;
  public static int fudicialId;
  public static double yaw;
  public static double pitch;
  public static boolean targetVisible;
  // private double target;
  // private int fudicialId;

  /** Creates a new PhotonVisonCamera. */
  public PhotonVisonCamera() {
    camera = new PhotonCamera("limelight_rear");
    this.yaw = 0.0;
    this.fudicialId = 0;
    this.pitch = 0.0;
    this.targetVisible = false;

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
    this.targetVisible = false;
    this.fu
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          this.fudicialId = target.getFiducialId();
          this.yaw = target.getYaw();
          this.pitch = target.getPitch(); 
          
          this.targetVisible = true;
        }
      }
    }

    SmartDashboard.putNumber("LimelightYaw ", this.yaw);
    SmartDashboard.putNumber("Id ", (double) this.fudicialId);
    SmartDashboard.putBoolean("IsVisible ", targetVisible);
  }
}
