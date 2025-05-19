// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkFlex topMotor;
  private final SparkFlex bottomMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    var config = new SparkFlexConfig();

    topMotor = new SparkFlex(9, MotorType.kBrushless);
    bottomMotor = new SparkFlex(8, MotorType.kBrushless);

    config.idleMode(IdleMode.kBrake);
  }

  public double getVelocity() {
    return topMotor.getEncoder().getVelocity();
  }

  public void runShooter(double value) {
    topMotor.set(value);
    bottomMotor.set(value);
  }

  public void stopShooter() {
    topMotor.set(0.0);
    bottomMotor.set(0.0);
  }

  // public void setSpeed(double speed) {
  //   shooterController.setReference(speed, ControlType.kVelocity);
  //   bottomMotor.follow(topMotor);
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", getVelocity());
  }
}
