// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// one motor
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.WristConstants;
/** Add your docs here. */
public class Wrist extends SubsystemBase {
    private final SparkMax wristMotor;
    private final PIDController wristPIDController;
    //private RelativeEncoder wristEncoder;
    private AbsoluteEncoder wristEncoder;
    private Trolley m_trolleyRef;
    private Pivot m_pivotRef;

    private static int kCPR = 8192; // Constant for Counts Per Revolution (CPR) from REV's website for REV Through Bore 

    public Wrist() {
        var config = new SparkMaxConfig();
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristPIDController = new PIDController(0.05, 0, 0);
        config.idleMode(IdleMode.kBrake);
        //wristEncoder = wristMotor.getAlternateEncoder(kCPR);
        wristEncoder = wristMotor.getAbsoluteEncoder();
        // wristPIDController.setP(0.05);
        // wristPIDController.setI(0);
        // wristPIDController.setD(0);
        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTrollyRef(Trolley trolleyRef)
    {
        m_trolleyRef = trolleyRef;
    }
    public void setPivotRef(Pivot pivotRef)
    {
        m_pivotRef = pivotRef;
    }
    public void wrist(double value) {
        wristMotor.set(value);
    }
    public void wristSTOP() {
        wristMotor.set(0);
    }
    public void wristController(double setpoint){
        wristMotor.set(wristPIDController.calculate(wristEncoder.getPosition(), setpoint));
    }
    public double getPosition(){return wristEncoder.getPosition();}
    public boolean atSetpoint(){
        if(wristEncoder.getPosition() == 0.122){
            return true;
        } else { return false;}
    }
    public boolean atSetpoint(double min, double max){
        return min<=wristEncoder.getPosition() && max>=wristEncoder.getPosition();
    }

    // SMART DASHBOARD 
       public boolean atIntakePositionWrist() {
        if (wristEncoder.getPosition() == WristConstants.INTAKE_POS) { 
          return true;
        } else {
            return false;
        }
      }

          public boolean atHomePositionWrist() {
        if (wristEncoder.getPosition() == WristConstants.HOME_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }

    // SMART DASHBOARD
    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderWrist" , wristEncoder.getPosition());
        SmartDashboard.putBoolean("WristAtIntakePosition", atIntakePositionWrist());
        SmartDashboard.putBoolean("WristAtHomePosition", atHomePositionWrist());
    }

    public boolean isWristAtMaxDown() { 
        return false;
    }
}