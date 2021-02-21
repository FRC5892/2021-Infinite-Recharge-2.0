// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Hood extends PIDSubsystem {
  VictorSP hoodMotor;
  DigitalInput bottomStop;
  DigitalInput topStop;
  AnalogInput hoodPotentiometer;
  /** Creates a new Hood. */
  public Hood() {
    super(
      // The PIDController used by the subsystem
      new PIDController(2, 0, 0));
      getController().setTolerance(1);
      hoodMotor = new VictorSP(Constants.Hood.HOOD_MOTOR_PORT);
      bottomStop = new DigitalInput(Constants.Hood.HOOD_BOTTOM_STOP);
      topStop = new DigitalInput(Constants.Hood.HOOD_TOP_STOP);
      hoodPotentiometer = new AnalogInput(Constants.Hood.HOOD_POTENTIOMETER);
  }

  public boolean getBottomStop() {
    return !bottomStop.get();
  }

  public boolean getTopStop() {
    return topStop.get();
  }

  public boolean atStop() {
    return !bottomStop.get() || topStop.get();
  }

  public double getHoodPotentiometer() {
    return hoodPotentiometer.getAverageVoltage();
  }
  
  public void setHoodVictorSPVictorSPPosition(double setpoint) {
    this.setSetpoint(setpoint);
  }
  
  public boolean goingForward() {
    if (!(this.getSetpoint() - getHoodPotentiometer() > 0)) {
      return false;
    }
    else {
      return false;
    }
  }
  public boolean atDirectionStop() {
    if (getTopStop()) {
      if (goingForward()) {
        return true;
      }
      else {
        return false;
      }
    }
    else if (getBottomStop()) {
      if (!goingForward()){
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }
  @Override
  public void useOutput(double output, double setpoint) {
    hoodMotor.set(output);
    System.out.println("output " + output);
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    return hoodPotentiometer.getAverageVoltage();
    //return 1;
  }
}
