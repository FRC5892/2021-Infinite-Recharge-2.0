// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbArm extends SubsystemBase {
  VictorSP armMotor;
  DoubleSolenoid armSolenoid;
  DigitalInput topLimit;
  DigitalInput bottomLimit;
 

  /** Creates a new Climb. */
  public ClimbArm() {
    armMotor = new VictorSP(Constants.Climb.CLIMB_ARM_MOTOR_PORT);
    armMotor.setInverted(true);
    armSolenoid = new DoubleSolenoid(Constants.Climb.CLIMB_ARM_SOLENOID_FORWARD, Constants.Climb.CLIMB_ARM_SOLENOID_REVERSE);
    armSolenoid.set(Value.kReverse);
    topLimit = new DigitalInput(Constants.Climb.CLIMB_TOP_LIMIT);
    bottomLimit = new DigitalInput(Constants.Climb.CLIMB_BOTTOM_LIMIT);
  }

  public void toggleArm() {
    armSolenoid.toggle();
  }

  public void setArmMotor(double speed) {
    if (!((speed > 0 && !topLimit.get()) || (speed < 0 && !bottomLimit.get()))) {
      System.out.println("Going");
      armMotor.set(speed);
    }
  }

  public void stopArmMotor() {
    armMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if ((armMotor.get() > 0 && !topLimit.get()) || (armMotor.get() < 0 && !bottomLimit.get())) {
      System.out.println("Stopping");
      armMotor.stopMotor();
    }
    // This method will be called once per scheduler run
  }
}
