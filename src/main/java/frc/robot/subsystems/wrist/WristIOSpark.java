// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.wrist.WristConstants.wristCanId;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class WristIOSpark implements WristIO {
  private final SparkFlex wristFlex = new SparkFlex(wristCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = wristFlex.getEncoder();
  private PIDController wristPIDController = new PIDController(1.25, 0, 0);
  private ArmFeedforward wristfeedForwardController = new ArmFeedforward(0.0, 0.13, 1.69, 4);

  public WristIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> wrist Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        wristFlex,
        5,
        () ->
            wristFlex.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    ifOk(wristFlex, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(wristFlex, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        wristFlex,
        new DoubleSupplier[] {wristFlex::getAppliedOutput, wristFlex::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(wristFlex, wristFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    wristFlex.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    wristFlex.set(speed);
  }

  @Override
  public void runWristPIDController(double sensor, double setPoint) {
    wristFlex.set(-wristPIDController.calculate(sensor, setPoint));
  }

  @Override
  public void runWristPIDControllerFeedForward(double setPoint) {
    wristFlex.setVoltage(
        wristPIDController.calculate(encoder.getPosition(), setPoint)
            + wristfeedForwardController.calculate(1.69, 0.04));
  }

  @Override
  public boolean atSetpoint() {
    return wristPIDController.atSetpoint();
  }
}
