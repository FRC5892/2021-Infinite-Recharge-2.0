// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbArm extends SubsystemBase {
  VictorSP armMotor;
  DoubleSolenoid armSolenoid;
 

  /** Creates a new Climb. */
  public ClimbArm() {
    armMotor = new VictorSP(Constants.Climb.CLIMB_ARM_MOTOR_PORT);
    armSolenoid = new DoubleSolenoid(Constants.Climb.CLIMB_ARM_SOLENOID_FORWARD, Constants.Climb.CLIMB_ARM_SOLENOID_REVERSE);
    armSolenoid.set(Value.kReverse);
  }

  public void toggleArm() {
    armSolenoid.toggle();
  }

  public void setArmMotor(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
