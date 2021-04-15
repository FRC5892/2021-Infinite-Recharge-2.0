// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Kicker;

public class RunKicker extends CommandBase {
	Kicker kicker;
	Timer timer;

	/** Creates a new RunKicker. */
	public RunKicker(Kicker k) {
		kicker = k;
		addRequirements(kicker);
		timer = new Timer();
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if ((RobotContainer.driverJoystick.getTriggerAxis(Hand.kLeft) != 0
				|| RobotContainer.driverJoystick.getTriggerAxis(Hand.kRight) != 0) && !kicker.ballLoaded()) {
			timer.reset();
			timer.start();
			kicker.setKicker(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
		}
		if (timer.get() > Constants.Kicker.KICKER_TIME_OUT) {
			kicker.stopKicker();
		}
		if (kicker.ballLoaded()) {
			kicker.stopKicker();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
