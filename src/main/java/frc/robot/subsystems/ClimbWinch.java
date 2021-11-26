// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbWinch extends SubsystemBase {
	VictorSP winchMotor;

	/** Creates a new ClimbWinch. */
	public ClimbWinch() {
		winchMotor = new VictorSP(Constants.Climb.CLIMB_WINCH_MOTOR_PORT);
	}

	public void setWinchMotor(double speed) {
		winchMotor.set(speed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
