// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  VictorSP rollerMotor;
  DoubleSolenoid leftDoubleSolenoid;
  DoubleSolenoid righDoubleSolenoid;

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new VictorSP(3);
    rollerMotor.setInverted(true);
    leftDoubleSolenoid = new DoubleSolenoid(2, 3);
    righDoubleSolenoid = new DoubleSolenoid(5, 6);
  }

  public void setRollersSpeed (double speed) {
    rollerMotor.setSpeed(speed);
  }

  public void stopRollers () {
    rollerMotor.stopMotor();
  }
  public void ToggleIntakePistons() {
    leftDoubleSolenoid.toggle();
    righDoubleSolenoid.toggle();
  }

  public void ExtendIntakePistons() {
    leftDoubleSolenoid.set(Value.kForward);
    righDoubleSolenoid.set(Value.kForward);
  }

  public void RetractIntakePistons() {
    leftDoubleSolenoid.set(Value.kReverse);
    righDoubleSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
