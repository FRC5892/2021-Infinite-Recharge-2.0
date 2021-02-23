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

  public void setHoodVictorSPVictorSPPosition(double setpoint) {
    this.setSetpoint(setpoint);
  }
  
  public boolean atDirectionStop() {
    return (hoodMotor.getSpeed()>0 && topStop.get())||(hoodMotor.getSpeed()<0 && !bottomStop.get());
  }

  public boolean atSetpoint() {
    return hoodPotentiometer.getAverageVoltage() == this.getSetpoint();
  }
  @Override
  public void useOutput(double output, double setpoint) {
    hoodMotor.set(output);
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    return hoodPotentiometer.getAverageVoltage();
  }
}
