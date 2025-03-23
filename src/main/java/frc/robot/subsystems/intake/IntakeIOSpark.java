// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  private SparkMax intake = new SparkMax(intakeCanId, MotorType.kBrushless);
  private RelativeEncoder encoder = intake.getEncoder();
  SparkClosedLoopController m_controller = intake.getClosedLoopController();
  private CANcoder intakeEncoder = new CANcoder(6);
  private TalonFXConfiguration newMagicMotion = new TalonFXConfiguration();


  // private PIDController pid = new PIDController(1.4, 0, 0);
  // private CANrange canRange = new CANrange(IntakeConstants.canRangeId);

  public IntakeIOSpark() {

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> intake Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config.closedLoop.p(0.8).i(0).d(1.0);

    var slot0Configs = newMagicMotion.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = newMagicMotion.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    tryUntilOk(
        intake,
        5,
        () ->
            intake.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(intake, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(intake, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        intake,
        new DoubleSupplier[] {intake::getAppliedOutput, intake::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(intake, intake::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.positionAbs = intakeEncoder.getAbsolutePosition().getValueAsDouble();

    inputs.inRange = inRange();
  }

  @Override
  public void setVoltage(double volts) {
    intake.setVoltage(volts);
  }

  @Override
  public void runWristPIDController(double sensor, double setPoint) {
    m_controller.setReference(setPoint, ControlType.kPosition, 0);
  }

  // @Override
  // public void setIntakeEncoderToZero() {
  //   encoder.setPosition(0);
  // }

  @Override
  public double getPosition() {
    return intakeEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // @Override
  // public void runWristPIDController(double sensor, double setPoint) {
  //   intake.set(pid.calculate(sensor, setPoint));
  // }

  //   @Override
  //   public boolean inRange() {
  //     return canRange.getIsDetected().getValue();
  //   }
}
