// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkFlex elevatorMotorLeader =
      new SparkFlex(elevatorMotorLeaderCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorFollower =
      new SparkFlex(elevatorMotorFollowerCanId, MotorType.kBrushless);
  private final SparkFlex elevatorMotorFollower2 =
      new SparkFlex(elevatorMotorFollower2CanId, MotorType.kBrushless);
  private final RelativeEncoder encoder1 = elevatorMotorLeader.getEncoder();
  private final RelativeEncoder encoder2 = elevatorMotorFollower.getEncoder();

  private final RelativeEncoder encoder =
      elevatorMotorLeader.getEncoder(); // it just doesnt work it shows red for some reason
  private PIDController elevatorPIDController = new PIDController(1.4, 0, 0);

  public ElevatorIOSpark() {

    // elevatorMotorLeader.restoreFactoryDefaults();
    // elevatorMotorFollower.restoreFactoryDefaults();

    // elevatorMotorLeader.setIdleMode(IdleMode.kBrake);
    // elevatorMotorFollower.setIdleMode(IdleMode.kBrake);

    // elevatorMotorLeader.burnFlash();
    // elevatorMotorFollower.burnFlash();

    // elevatorPIDController = elevatorMotorLeader.getEncoder();
    // RelativeEncoder = elevatorMotorLeader.getEncoder();

    // elevatorPIDController.setFeedbackDevice(alternateEncoder);
    // elevatorPIDController.setP(0.1);
    // elevatorPIDController.setI(0.0);
    // elevatorPIDController.setD(0.0);
    // elevatorPIDController.setFF(0.0);
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> intake Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        elevatorMotorLeader,
        5,
        () ->
            elevatorMotorLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        elevatorMotorFollower,
        5,
        () ->
            elevatorMotorFollower.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        config.follow(elevatorMotorLeader);

    tryUntilOk(
          elevatorMotorFollower2,
          5,
          () ->
              elevatorMotorFollower2.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
          config.follow(elevatorMotorFollower);

  }


  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(elevatorMotorLeader, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotorLeader, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorLeader,
        new DoubleSupplier[] {
          elevatorMotorLeader::getAppliedOutput, elevatorMotorLeader::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorLeader,
        elevatorMotorLeader::getOutputCurrent,
        (value) -> inputs.currentAmps = value);

    ifOk(elevatorMotorFollower, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(elevatorMotorFollower, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        elevatorMotorFollower,
        new DoubleSupplier[] {
          elevatorMotorFollower::getAppliedOutput, elevatorMotorFollower::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        elevatorMotorFollower,
        elevatorMotorFollower::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotorLeader.setVoltage(volts);
    //elevatorMotorFollower.setVoltage(volts);
  }

  @Override
  public void setValue(double value){
    encoder1.setPosition(value);
  }

  @Override
  public void runElevatorPIDController( double setPoint) {
    elevatorMotorLeader.set(elevatorPIDController.calculate(encoder1.getPosition(), setPoint));
  }
}
