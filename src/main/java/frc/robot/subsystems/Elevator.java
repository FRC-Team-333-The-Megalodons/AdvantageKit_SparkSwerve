// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkFlex elevatorMotor1;

  private SparkFlex elevatorMotor2;
  private DigitalInput elevatorSensor;

  public Elevator() {
    elevatorMotor1 = new SparkFlex(8, MotorType.kBrushless);
    elevatorMotor2 = new SparkFlex(9, MotorType.kBrushless);
    elevatorSensor = new DigitalInput(0);
  }

  public void setSpeed(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
  }

  public boolean getLimitSwitchState() {
    return elevatorSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
