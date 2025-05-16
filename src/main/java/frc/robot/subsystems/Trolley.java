// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrolleyConstants;

public class Trolley extends SubsystemBase {

  private Pivot pivotRef;
  private Wrist wristRef;

 private SparkFlex trolleyMotor = new SparkFlex(0, MotorType.kBrushless);
  private final RelativeEncoder internalEncoder = trolleyMotor.getEncoder();
  private final DutyCycleEncoder externalEncoder = new DutyCycleEncoder(0);
  private final PIDController trolleyController = new PIDController(TrolleyConstants.kP, TrolleyConstants.kI, TrolleyConstants.kD);
  private PIDController trolleyPIDController;
  private DigitalInput maxOutLimitSwitch, minInLimitSwitch;

  
  private AnalogInput potInput;
  private AnalogPotentiometer potentiometer;

  public Trolley() {
    var config = new SparkFlexConfig();
    trolleyMotor = new SparkFlex(TrolleyConstants.TROLLEY_MOTOR_ID, MotorType.kBrushless);
    trolleyPIDController = new PIDController(TrolleyConstants.kP, TrolleyConstants.kI, TrolleyConstants.kD);
    config.idleMode(IdleMode.kBrake);
    maxOutLimitSwitch = new DigitalInput(TrolleyConstants.TROLLEY_OUT_LIMIT_SWITCH_ID);
    minInLimitSwitch = new DigitalInput(TrolleyConstants.TROLLEY_IN_LIMIT_SWITCH_ID);


    // Removed restoreFactoryDefaults() as it is not defined for SparkFlex

    // trolleyController.setP(TrolleyConstants.kP);
    // trolleyController.setI(TrolleyConstants.kI);
    // trolleyController.setD(TrolleyConstants.kD);
    // trolleyController.setOutputRange(TrolleyConstants.MIN_INPUT, TrolleyConstants.MAX_INPUT);

    //SparkFlexConfig config = trolleyMotor.getConfig();
    //trolleyMotor.setIdleMode(IdleMode.kBrake);

 //   trolleyMotor.burnFlash();

  }

  public void setPivotRef(Pivot _pivotRef) {
    pivotRef = _pivotRef;
  }

  public void setWristRef(Wrist _wristRef) {
    wristRef = _wristRef;
  }

  public void resetEncoder() {
    trolleyMotor.getEncoder().setPosition(0.0);
  }

  public void runTrolley(double speed) {
    // Negative number means moving trolley in; positive number means moving trolley out.
    if (speed > 0) {
      if (isTrolleyAtMaxOutLimitSwitch()) {
        stopTrolley();
        return;
      }
    } else if (speed < 0) {
      if (isTrolleyAtMinInLimitSwitch()) {
        stopTrolley();
        return;
      }
    }
    trolleyMotor.set(speed);
  }

  public void stopTrolley() {
    trolleyMotor.set(0.0);
  }

  public void setPosition(double setpoint) {
    trolleyController.setSetpoint(setpoint);
  }

  public boolean atSetpoint(double setpoint) {
    // If our encoder is at a premeditated setpoint, return true, otherwise return false
    return (getPotentiometerPosition() == setpoint);
  }

  public boolean fuzzyEquals(double a, double b) {
    final double epsilon = 0.01;
    return Math.abs(a-b) < epsilon;
  }

  public double getPotentiometerPosition() {
      // We flip the sign, add a constant, and multiply by 100 to
      //  make this number more "intuitive" / legible.
      return potentiometer.get() * -100 + 7;
  }

  public boolean isOkToMoveTrolleyOut() {
    if (isTrolleyAtMaxOutLimitSwitch()) {
      return false;
    }
    return true;
  }

  public boolean isOkToMoveTrolleyIn() {
    if (isTrolleyAtMinInLimitSwitch()) {
      return false;
    }
    return true;
  }
  
  public boolean isTrolleyAtMaxOutLimitSwitch() {
    return !maxOutLimitSwitch.get();
  }

  public boolean isTrolleyAtMinInLimitSwitch() {
    if (!minInLimitSwitch.get()) {
      resetEncoder();
    }
    return !minInLimitSwitch.get();
  }

  // This function returns whether the trolley is "past the frame perimeter".
  // This could technically vary based on where the wrist is, but for now we'll just use a single value.
  public boolean isTrolleyOut() {
    // return getPotentiometerPosition() >= TrolleyConstants.TROLLEY_IN_OUT_THRESHOLD;
    // workaround till pot is online
    return !isTrolleyAtMinInLimitSwitch();
  }

  public boolean isTrolleyIn() {
    return !isTrolleyOut();
  }

  public boolean isTrolleyTooFarInToPivotVertical() {
    return getPotentiometerPosition() < TrolleyConstants.TROLLEY_FURTHEST_IN_WHERE_PIVOT_CAN_MOVE_ALL_THE_WAY_UP;
  }

  public boolean isTrolleyTooFarInToPivotUpPastBumper() {
    return getPotentiometerPosition() < TrolleyConstants.TROLLEY_FURTHEST_IN_WHERE_PIVOT_CAN_CLEAR_BACK_BUMPER;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("TrolleyMaxOutLimit", isTrolleyAtMaxOutLimitSwitch());
    SmartDashboard.putBoolean("TrolleyMinInLimit", isTrolleyAtMinInLimitSwitch());
    SmartDashboard.putNumber("TrolleyEncoder" , trolleyMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("TrolleyAtSetpoint", atSetpoint(getPotentiometerPosition()));
  }
}
