// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
	private final DriveTrain driveTrain;

	/** Creates a new DriveWithJoysticks. */
	public DriveWithJoysticks(DriveTrain dt) {
		// Use addRequirements() here to declare subsystem dependencies.
		driveTrain = dt;
		addRequirements(driveTrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (OperatorInput.driverJoystick.getBumper(Hand.kRight)) {
			driveTrain.driveWithJoysticks(OperatorInput.driverJoystick, Constants.DriveTrain.SLOW_SPEED,
					Constants.DriveTrain.SLOW_SPEED);
		}
		else {
			driveTrain.driveWithJoysticks(OperatorInput.driverJoystick, Constants.DriveTrain.SPEED,
					Constants.DriveTrain.SPEED);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveTrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
