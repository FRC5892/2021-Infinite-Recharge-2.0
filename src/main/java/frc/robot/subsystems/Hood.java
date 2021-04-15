// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Hood extends PIDSubsystem {
	VictorSP hoodMotor;
	DigitalInput bottomStop;
	DigitalInput topStop;
	AnalogInput hoodPotentiometer;
	NetworkTableInstance networkTableInstance;
	NetworkTable smartdashboardTable;

	/** Creates a new Hood. */
	public Hood() {
		super(
				// The PIDController used by the subsystem
				new PIDController(0.5, 0, 0));
		getController().setTolerance(0.5);
		hoodMotor = new VictorSP(Constants.Hood.HOOD_MOTOR_PORT);
		bottomStop = new DigitalInput(Constants.Hood.HOOD_BOTTOM_STOP);
		topStop = new DigitalInput(Constants.Hood.HOOD_TOP_STOP);
		hoodPotentiometer = new AnalogInput(Constants.Hood.HOOD_POTENTIOMETER);
	}

	public double getHoodAngle() {
		return hoodPotentiometer.getAverageVoltage() * (-75.5467) + 239.459;
	}

	public void setHood(double setpoint) {
		this.setSetpoint(setpoint);
		this.enable();
	}

	public boolean atDirectionStop() {
		return (hoodMotor.get() > 0 && topStop.get()) || (hoodMotor.get() < 0 && bottomStop.get());
	}

	public boolean atSetpoint() {
		return this.getController().atSetpoint();
	}

	@Override
	public void useOutput(double output, double setpoint) {
		if (!atDirectionStop()) {
			hoodMotor.set(output);
		}
		else {
			this.disable();
		}
		SmartDashboard.putNumber("Hood Setpoint", this.getSetpoint());
		SmartDashboard.setDefaultNumber("Set Hood Angle", 0);
		SmartDashboard.putNumber("Hood Potentiometer Angle", getHoodAngle());
		SmartDashboard.putNumber("Hood Potentiometer", hoodPotentiometer.getAverageVoltage());
		SmartDashboard.putBoolean("Hood At Endstop", atDirectionStop());
		// Use the output here
	}

	@Override
	public double getMeasurement() {
		return getHoodAngle();
	}
}
