// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Kicker extends SubsystemBase {
	VictorSP kickerMotor;
	DigitalInput kickerSensor;

	/** Creates a new Kicker. */
	public Kicker() {
		kickerMotor = new VictorSP(Constants.Kicker.KICKER_MOTOR_PORT);
		kickerMotor.setInverted(true);
		kickerSensor = new DigitalInput(Constants.Kicker.KICKER_SENSOR_PORT);
	}

	public void setKicker(double speed) {
		kickerMotor.set(speed);
	}

	public void stopKicker() {
		kickerMotor.stopMotor();
	}

	public boolean ballLoaded() {
		return kickerSensor.get();
	}

	@Override
	public void periodic() {
		RobotContainer.driverJoystick.setRumble(RumbleType.kLeftRumble, kickerMotor.get());
		SmartDashboard.putBoolean("Ball Loaded", this.ballLoaded());
		SmartDashboard.putNumber("Kicker Speed", kickerMotor.getSpeed());
		// This method will be called once per scheduler run
	}
}
