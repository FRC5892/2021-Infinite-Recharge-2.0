// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Accumulator extends SubsystemBase {
  VictorSP accumulatorMotor;
  
  /** Creates a new Accumulator. */
  public Accumulator() {
    accumulatorMotor = new VictorSP(Constants.Accumulator.ACCUMULATOR_MOTOR_PORT);
  }

  public void startAccumulator() {
    accumulatorMotor.setSpeed(Constants.Accumulator.ACCUMULATOR_MOTOR_SPEED);
  }

  public void stopAccumulator() {
    accumulatorMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
