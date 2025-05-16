// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private DigitalInput peLeft;
  private DigitalInput peRight;
  private final SparkFlex intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new SparkFlex(6, MotorType.kBrushless);
    //  intakeMotor.setIdleMode(IdleMode.kCoast);
    peLeft = new DigitalInput(IntakeConstants.LEFT_SENSOR_ID);
    peRight = new DigitalInput(IntakeConstants.RIGHT_SENSOR_ID);
  }

  public void intake(double value) {
    intakeMotor.set(value);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public boolean detectNote() {
    if (peLeft.get() || peRight.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean detectRight() {
    if (peRight.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean detectLeft() {
    if (peLeft.get()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("GetNote", detectNote());
    SmartDashboard.putBoolean("GetRight", detectLeft());
    SmartDashboard.putBoolean("GetLeft", detectRight());
  }
}
