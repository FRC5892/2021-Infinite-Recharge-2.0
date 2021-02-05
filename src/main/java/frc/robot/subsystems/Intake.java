// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  VictorSP rollerMotor;
  DoubleSolenoid solenoid;

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new VictorSP(3);
    rollerMotor.setInverted(true);
    solenoid = new DoubleSolenoid(Constants.Intake.SOLENOID_FORWARD, Constants.Intake.SOLENOID_REVERSE);
    solenoid.set(Value.kReverse);
  }

  public void setRollersSpeed (double speed) {
    rollerMotor.setSpeed(speed);
  }

  public void stopRollers () {
    rollerMotor.stopMotor();
  }
  public void toggleIntakePistons() {
    solenoid.toggle();
  }

  public void extendIntakePistons() {
    solenoid.set(Value.kForward);
  }

  public void retractIntakePistons() {
    solenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
