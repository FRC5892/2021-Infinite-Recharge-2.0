// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  VictorSP kickerMotor;
  /** Creates a new Kicker. */
  public Kicker() {
    kickerMotor = new VictorSP(Constants.Kicker.KICKER_MOTOR_PORT);
    kickerMotor.setInverted(true);
  }

  public void startKicker() {
    kickerMotor.set(Constants.Kicker.KICKER_MOTOR_SPEED);
  }

  public void stopKicker() {
    kickerMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
