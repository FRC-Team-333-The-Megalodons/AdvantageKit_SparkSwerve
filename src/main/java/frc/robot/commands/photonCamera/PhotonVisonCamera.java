// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class PhotonVisonCamera extends SubsystemBase {
  private PhotonCamera camera;
  /** Creates a new PhotonVisonCamera. */
  public PhotonVisonCamera() {
    camera = new PhotonCamera("limelight_rear");
  }

  public double getYaw() {
    var results = camera.getAllUnreadResults();
    double yaw = 0;
    for (var result : results) {
      if (result.hasTargets()) {
        yaw = result.getBestTarget().getYaw();
      }
    }
    return yaw;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lime light Yaw", getYaw());
  }
}
